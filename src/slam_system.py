"""SLAM system with background thread for web control.

Runs a stop-and-scan loop in a background thread:
  - Wait for target
  - Move to target (blocking)
  - Scan + ICP correct + update map
  - Repeat
"""

import math
import threading
import time
import traceback

import lgpio
import numpy as np

from frontier import find_frontiers, get_frontier_viz_data, select_goal
from icp import icp
from occupancy_grid import OccupancyGrid, scan_to_world
from path_planner import plan_and_smooth
from robot.drive_dc import RobotDC
from scanner import Scanner


class SlamSystem:
    def __init__(self, use_icp=True):
        self.use_icp = use_icp

        # Shared state (GIL-safe for simple reads/writes)
        self.state = "IDLE"  # IDLE, MOVING, SCANNING, ERROR
        self.pose = (0.0, 0.0, 0.0)  # x, y, heading_deg
        self.target = None  # (x, y) or None
        self.message = "Initializing..."
        self.map_version = 0
        self.icp_result = None  # dict with last ICP result info
        self.path_history = [(0.0, 0.0)]  # list of (x, y) visited
        self.icp_corrections = []  # list of {"from": [x,y], "to": [x,y]}
        self.pid_summary = None  # last movement PID stats
        self.planned_waypoints = []  # current planned path [(x,y), ...]

        # Exploration state
        self._exploring = False
        self.explore_goal = None  # current (x, y) exploration target
        self.frontier_data = []  # viz data for web UI

        # Hardware
        self.chip = None
        self.robot = None
        self.scanner = None
        self.grid = OccupancyGrid()

        self._running = False
        self._thread = None

    def start(self):
        """Start background control loop."""
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop and cleanup."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.robot:
            self.robot.close()
        if self.chip:
            lgpio.gpiochip_close(self.chip)

    def set_target(self, x, y):
        """Set navigation target. Returns False if busy."""
        if self.state != "IDLE":
            return False
        self.target = (x, y)
        self.message = f"Target set: ({x:.1f}, {y:.1f})"
        return True

    def get_map_data(self):
        """Return cropped map data for the frontend.

        Returns dict with:
          grid: 2D list of probabilities (0=free, 1=occupied)
          traversable: 2D list of booleans (True=safe to drive)
          bounds: [x_min, x_max, y_min, y_max] in cm
          rows, cols: dimensions
        """
        prob_map = self.grid.get_probability_map()
        r0, r1, c0, c1 = self.grid._get_data_bounds(padding=10)
        cropped = prob_map[r0 : r1 + 1, c0 : c1 + 1]

        x_min, y_min = self.grid.grid_to_world(r0, c0)
        x_max, y_max = self.grid.grid_to_world(r1, c1)

        # Traversability overlay
        try:
            trav = self.grid.get_traversability_grid()
            trav_cropped = trav[r0 : r1 + 1, c0 : c1 + 1].tolist()
        except Exception:
            trav_cropped = None

        return {
            "grid": np.round(cropped, 2).tolist(),
            "traversable": trav_cropped,
            "bounds": [x_min, x_max, y_min, y_max],
            "rows": cropped.shape[0],
            "cols": cropped.shape[1],
            "version": self.map_version,
        }

    # ── Background loop ───────────────────────────────────────────────

    def _loop(self):
        """Main background loop."""
        self._init_hardware()

        while self._running:
            if self._exploring:
                self._explore_loop()
            elif self.target:
                target = self.target
                self.target = None
                self._move_scan_update(target)
            time.sleep(0.1)

    def _init_hardware(self):
        """Initialize hardware, take initial scan."""
        try:
            print("Initializing hardware...")
            self.chip = lgpio.gpiochip_open(4)
            self.robot = RobotDC(self.chip)
            self.scanner = Scanner()

            # Initial scan at origin
            self._scan_and_update()
            self.state = "IDLE"
            self.message = "Ready"
            print("Hardware ready.")

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Init error: {e}"
            traceback.print_exc()

    def _move_scan_update(self, target):
        """Navigate to target using receding horizon path planning.

        At each step: plan path → follow first few waypoints → scan → repeat.
        Re-plans from current position every iteration to handle new obstacles.
        """
        tx, ty = target
        ARRIVAL_THRESHOLD = 10.0  # cm — close enough to target
        MAX_STEPS = 50  # safety limit
        MAX_WPS_PER_MOVE = 3  # follow this many waypoints before scanning

        for step_i in range(MAX_STEPS):
            # Check if we've arrived
            dist_to_goal = math.hypot(tx - self.pose[0], ty - self.pose[1])
            if dist_to_goal < ARRIVAL_THRESHOLD:
                self.message = f"Arrived at ({tx:.0f}, {ty:.0f})!"
                print(f"Arrived at target (dist={dist_to_goal:.1f}cm)")
                break

            # Plan path from current pose to target
            self.state = "PLANNING"
            self.message = f"Planning path to ({tx:.0f}, {ty:.0f})..."
            start_xy = (self.pose[0], self.pose[1])
            waypoints = plan_and_smooth(self.grid, start_xy, (tx, ty))

            if waypoints is None or len(waypoints) < 2:
                self.message = "No path found!"
                print(
                    f"No path to ({tx:.0f}, {ty:.0f}) from "
                    f"({start_xy[0]:.0f}, {start_xy[1]:.0f})"
                )
                break

            # On first step: check if path starts with a sharp turn → back up
            if step_i == 0 and len(waypoints) >= 2:
                wp1x, wp1y = waypoints[1]
                angle_to_wp1 = math.degrees(
                    math.atan2(wp1y - self.pose[1], wp1x - self.pose[0])
                )
                wp1_diff = abs((angle_to_wp1 - self.pose[2] + 180) % 360 - 180)
                if wp1_diff > self.SHARP_TURN_DEG:
                    left_dist = self._get_clearance(self.pose[2] + 90, 40.0)
                    right_dist = self._get_clearance(self.pose[2] - 90, 40.0)

                    if left_dist > right_dist + 5.0:
                        print(
                            f"Sharp first waypoint ({wp1_diff:.0f}°) — Left clearer, turning nose left"
                        )
                        self._back_up_arc(-25.0, -15.0)
                    else:
                        # Default to favoring the right side if equal or right is clearer
                        print(
                            f"Sharp first waypoint ({wp1_diff:.0f}°) — Right clearer (or tie), turning nose right"
                        )
                        self._back_up_arc(25.0, -15.0)
                    continue  # re-plan from new position

            # Store full planned path for UI display
            self.planned_waypoints = [(round(x, 1), round(y, 1)) for x, y in waypoints]

            # Follow a chunk of waypoints (include start position + next N)
            chunk = waypoints[: MAX_WPS_PER_MOVE + 1]
            print(
                f"Step {step_i + 1}: planned {len(waypoints)} waypoints, "
                f"following {len(chunk) - 1}"
            )

            # Execute movement along the path chunk
            self.state = "MOVING"
            self.message = "Calibrating gyro..."
            try:
                self.robot.imu.calibrate_gyro(samples=100)
                self.message = f"Following path ({len(chunk) - 1} waypoints)..."
                self.robot.history = []
                self.robot.follow_path(chunk)
                self.pose = self.robot.get_pose()
                self.path_history.append((self.pose[0], self.pose[1]))

                if self.robot.history:
                    h = self.robot.history
                    step = max(1, len(h) // 100)
                    self.pid_summary = {
                        "t": [round(s["t"], 3) for s in h[::step]],
                        "left_pwm": [round(s["left_pwm"]) for s in h[::step]],
                        "right_pwm": [round(s["right_pwm"]) for s in h[::step]],
                        "heading_error": [
                            round(s["heading_error"], 1) for s in h[::step]
                        ],
                    }
            except Exception as e:
                self.state = "ERROR"
                self.message = f"Move failed: {e}"
                traceback.print_exc()
                break

            # Scan and update map
            odom_pos = (self.pose[0], self.pose[1])
            self._scan_and_update()

            # Record ICP correction if it happened
            corrected_pos = (self.pose[0], self.pose[1])
            if self.icp_result and self.icp_result.get("status") == "converged":
                self.path_history[-1] = corrected_pos
                self.icp_corrections.append(
                    {
                        "from": [round(odom_pos[0], 1), round(odom_pos[1], 1)],
                        "to": [
                            round(corrected_pos[0], 1),
                            round(corrected_pos[1], 1),
                        ],
                    }
                )

        self.planned_waypoints = []  # clear after navigation
        if not self._exploring:
            self.state = "IDLE"
            self.message = "Ready"

    def _scan_and_update(self):
        """Scan, optionally ICP correct, update grid."""
        self.state = "SCANNING"
        self.message = "Scanning..."

        try:
            scan = self.scanner.scan()
            pose = self.robot.get_pose()

            # ICP correction: scan-to-map matching
            if self.use_icp and len(scan) > 5:
                map_points = self.grid.get_occupied_points()

                # Only match against nearby map points (within 200cm of robot)
                # Prevents cross-wall matches when pose estimate is slightly off
                if len(map_points) > 0:
                    dist_to_robot = np.linalg.norm(
                        map_points - np.array([pose[0], pose[1]]), axis=1
                    )
                    map_points = map_points[dist_to_robot < 200]

                if len(map_points) > 10:
                    scan_world, _ = scan_to_world(scan, pose)
                    R, t, _, ok = icp(scan_world, map_points, max_distance=8)

                    if ok:
                        corr_pos = R @ np.array([pose[0], pose[1]]) + t
                        corr_angle = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                        dx = corr_pos[0] - pose[0]
                        dy = corr_pos[1] - pose[1]

                        # Sanity cap: reject implausibly large corrections
                        # These are almost always bad matches, not real drift
                        correction_dist = math.hypot(dx, dy)
                        if correction_dist > 10.0 or abs(corr_angle) > 10.0:
                            self.icp_result = {
                                "status": "rejected",
                                "dx": round(dx, 1),
                                "dy": round(dy, 1),
                                "dtheta": round(corr_angle, 1),
                                "map_pts": len(map_points),
                            }
                            print(
                                f"ICP rejected: dx={dx:.1f} dy={dy:.1f} dθ={corr_angle:.1f}° "
                                f"(too large, map: {len(map_points)} pts)"
                            )
                        else:
                            self.robot.set_pose(
                                corr_pos[0], corr_pos[1], pose[2] + corr_angle
                            )
                            pose = self.robot.get_pose()
                            self.icp_result = {
                                "status": "converged",
                                "dx": round(dx, 1),
                                "dy": round(dy, 1),
                                "dtheta": round(corr_angle, 1),
                                "map_pts": len(map_points),
                            }
                            print(
                                f"ICP converged: dx={dx:.1f} dy={dy:.1f} dθ={corr_angle:.1f}° (map: {len(map_points)} pts)"
                            )
                    else:
                        self.icp_result = {
                            "status": "failed",
                            "map_pts": len(map_points),
                        }
                        print(f"ICP failed to converge (map: {len(map_points)} pts)")
                else:
                    self.icp_result = {"status": "building_map"}

            # Update grid
            self.grid.update(scan, pose)
            self.pose = pose
            self.map_version += 1

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Scan error: {e}"
            traceback.print_exc()

    # ── Exploration ─────────────────────────────────────────────────

    # Constants for explore loop
    BOOTSTRAP_DRIVE_CM = 15.0  # Forward drive when map is too young for frontiers
    MIN_FRONTIER_DIST = 15.0  # Below this, drive forward instead of path-planning
    SHARP_TURN_DEG = 80.0  # Back up first if heading change exceeds this
    STRONGLY_BEHIND_DEG = 150.0  # Turn in place first if goal is this far off heading
    BACKUP_CM = 20.0  # How far to back up before a sharp turn

    def explore(self):
        """Start autonomous frontier exploration."""
        if self._exploring or self.state != "IDLE":
            return False
        self._exploring = True
        self.message = "Starting exploration..."
        return True

    def stop_explore(self):
        """Stop autonomous exploration after current move."""
        self._exploring = False
        self.explore_goal = None
        self.frontier_data = []
        self.robot.stop()  # Immediately halt any physical movement
        self.state = "IDLE"
        self.message = "Exploration stopped"

    def _explore_loop(self):
        """One iteration: detect frontiers → select goal → navigate → scan."""
        self.state = "EXPLORING"
        self.message = "Detecting frontiers..."
        clusters = find_frontiers(self.grid)
        self.frontier_data = get_frontier_viz_data(self.grid, clusters, self.pose)

        # No frontiers found
        if not clusters:
            if self.map_version <= 2:
                # Early map — drive forward to bootstrap coverage
                print("No frontiers yet (early map) — driving forward")
                self._drive_forward_safely(self.BOOTSTRAP_DRIVE_CM)
                return
            self._exploring = False
            self.explore_goal = None
            self.state = "IDLE"
            self.message = "Exploration complete — no frontiers remain!"
            print("Exploration complete: no frontiers found.")
            return

        print(
            f"Found {len(clusters)} frontier clusters "
            f"(sizes: {[len(c) for c in clusters[:5]]})"
        )

        # Select best reachable goal
        goal = select_goal(self.grid, clusters, self.pose)
        if goal is None:
            if self.map_version <= 2:
                print(
                    "No reachable frontiers (early map) — driving forward to bootstrap"
                )
                self._drive_forward_safely(self.BOOTSTRAP_DRIVE_CM)
                return

            self._exploring = False
            self.explore_goal = None
            self.state = "IDLE"
            self.message = "No reachable frontiers — exploration stopped."
            print("No reachable frontiers.")
            return

        gx, gy = goal
        dist = math.hypot(gx - self.pose[0], gy - self.pose[1])
        print(f"Explore goal: ({gx:.0f}, {gy:.0f}), dist={dist:.0f}cm")

        if dist < self.MIN_FRONTIER_DIST:
            print(f"Frontier too close ({dist:.0f}cm) — driving forward to get better angle")
            heading_rad = math.radians(self.pose[2])
            check_x = self.pose[0] + self.BOOTSTRAP_DRIVE_CM * math.cos(heading_rad)
            check_y = self.pose[1] + self.BOOTSTRAP_DRIVE_CM * math.sin(heading_rad)
            cr, cc = self.grid.world_to_grid(check_x, check_y)
            traversable = self.grid.get_traversability_grid()
            wall_ahead = (
                0 <= cr < traversable.shape[0]
                and 0 <= cc < traversable.shape[1]
                and not traversable[cr, cc]
            )
            if wall_ahead:
                left_dist = self._get_clearance(self.pose[2] + 90, 40.0)
                right_dist = self._get_clearance(self.pose[2] - 90, 40.0)
                if left_dist > right_dist + 5.0:
                    print("Wall ahead and frontier too close — arcing left")
                    self._back_up_arc(-25.0, -15.0)
                else:
                    print("Wall ahead and frontier too close — arcing right")
                    self._back_up_arc(25.0, -15.0)
            else:
                self._drive_forward_safely(self.BOOTSTRAP_DRIVE_CM)
            return

        # Navigate to goal
        self.explore_goal = goal
        self.message = f"Exploring → ({gx:.0f}, {gy:.0f})"

        # Back up if goal is nearly behind us — creates room for the arc
        angle = math.degrees(math.atan2(gy - self.pose[1], gx - self.pose[0]))
        heading_diff = abs((angle - self.pose[2] + 180) % 360 - 180)
        if heading_diff > self.STRONGLY_BEHIND_DEG:
            left_dist = self._get_clearance(self.pose[2] + 90, 40.0)
            right_dist = self._get_clearance(self.pose[2] - 90, 40.0)

            if left_dist > right_dist + 5.0:
                print(
                    f"Goal behind ({heading_diff:.0f}°) — Left clearer, turning nose left"
                )
                self._back_up_arc(-25.0, -20.0)
            else:
                # Default to favoring the right side if equal or right is clearer
                print(
                    f"Goal behind ({heading_diff:.0f}°) — Right clearer (or tie), turning nose right"
                )
                self._back_up_arc(25.0, -20.0)
        self._move_scan_update(goal)
        self.explore_goal = None

    def _drive_forward_safely(self, distance_cm: float):
        """Drive forward (with wall check) then scan."""
        heading_rad = math.radians(self.pose[2])
        check_x = self.pose[0] + distance_cm * math.cos(heading_rad)
        check_y = self.pose[1] + distance_cm * math.sin(heading_rad)
        cr, cc = self.grid.world_to_grid(check_x, check_y)
        traversable = self.grid.get_traversability_grid()
        if (
            0 <= cr < traversable.shape[0]
            and 0 <= cc < traversable.shape[1]
            and not traversable[cr, cc]
        ):
            print("Wall ahead — skipping forward drive")
            self._scan_and_update()
            return

        try:
            self.state = "MOVING"
            self.robot.imu.calibrate_gyro(samples=100)
            self.robot.history = []
            self.robot.forward(distance_cm)
            self.pose = self.robot.get_pose()
            self.path_history.append((self.pose[0], self.pose[1]))
        except Exception as e:
            self.state = "ERROR"
            self.message = f"Move failed: {e}"
            traceback.print_exc()
            self._exploring = False
            return
        self._scan_and_update()

    def _back_up_arc(self, radius_cm: float, arc_length_cm: float):
        """Back up in an arc to swing the nose into free space."""
        try:
            self.state = "MOVING"
            self.robot.imu.calibrate_gyro(samples=100)
            self.robot.history = []
            self.robot.arc(radius_cm, arc_length_cm)
            self.pose = self.robot.get_pose()
            self.path_history.append((self.pose[0], self.pose[1]))
        except Exception as e:
            print(f"Backup arc failed: {e}")

    def _get_clearance(self, angle_deg: float, max_dist_cm: float = 40.0) -> float:
        """Raycast on the grid to find free distance along 'angle_deg'."""
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        step_cm = 2.0
        dist = 0.0
        grid_data = self.grid.grid
        rows, cols = grid_data.shape

        while dist < max_dist_cm:
            cx = self.pose[0] + dist * cos_a
            cy = self.pose[1] + dist * sin_a
            r, c = self.grid.world_to_grid(cx, cy)

            if not (0 <= r < rows and 0 <= c < cols):
                break
            if grid_data[r, c] >= 0.5:  # Obstacle or Unknown
                break
            dist += step_cm

        return dist
