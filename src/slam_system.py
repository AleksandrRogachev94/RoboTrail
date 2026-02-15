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

from icp import icp
from occupancy_grid import OccupancyGrid
from robot.config import TOF_OFFSET_X, TOF_OFFSET_Y
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

        # Hardware
        self.chip = None
        self.robot = None
        self.scanner = None
        self.grid = OccupancyGrid()
        self.prev_scan_world = None

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
          bounds: [x_min, x_max, y_min, y_max] in cm
          rows, cols: dimensions
        """
        prob_map = self.grid.get_probability_map()
        r0, r1, c0, c1 = self.grid._get_data_bounds(padding=10)
        cropped = prob_map[r0 : r1 + 1, c0 : c1 + 1]

        x_min, y_min = self.grid.grid_to_world(r0, c0)
        x_max, y_max = self.grid.grid_to_world(r1, c1)

        return {
            "grid": np.round(cropped, 2).tolist(),
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
            if self.target:
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

    MAX_ARC_ANGLE_DEG = 30  # scan every ~30° of heading change

    def _move_scan_update(self, target):
        """Move to target with intermediate scans for large heading changes."""
        tx, ty = target

        self.state = "MOVING"
        try:
            # ZUPT before first move
            self.message = "Calibrating gyro..."
            self.robot.imu.calibrate_gyro(samples=100)

            # Check arc angle to target
            result = self.robot.arc_to_point(tx, ty)

            if result[0] is not None:
                radius, arc_length = result
                arc_angle_deg = abs(math.degrees(arc_length / radius))
            else:
                arc_angle_deg = 0  # straight line

            if arc_angle_deg > self.MAX_ARC_ANGLE_DEG:
                # Large heading change — use intermediate waypoints
                n_segments = math.ceil(arc_angle_deg / self.MAX_ARC_ANGLE_DEG)
                print(
                    f"Large arc ({arc_angle_deg:.0f}°), splitting into {n_segments} segments"
                )

                sx, sy = self.robot.x, self.robot.y  # start position
                for i in range(1, n_segments + 1):
                    if i < n_segments:
                        # Intermediate waypoint: fraction of the way from start to target
                        frac = i / n_segments
                        wx = sx + frac * (tx - sx)
                        wy = sy + frac * (ty - sy)
                        self._do_move(wx, wy)
                        self._scan_and_update()
                        # ZUPT before next segment
                        self.state = "MOVING"
                        self.message = "Calibrating gyro..."
                        self.robot.imu.calibrate_gyro(samples=100)
                    else:
                        self._do_move(tx, ty)
            else:
                # Small arc or straight — one move
                self._do_move(tx, ty)

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Move failed: {e}"
            traceback.print_exc()
            return

        # Final scan
        self._scan_and_update()
        self.state = "IDLE"
        self.message = "Ready"

    def _do_move(self, x, y):
        """Execute a single move_to and update pose."""
        self.message = f"Moving to ({x:.1f}, {y:.1f})..."
        print(f"Moving to ({x:.1f}, {y:.1f})...")
        self.robot.move_to(x, y)
        self.pose = self.robot.get_pose()

    def _scan_and_update(self):
        """Scan, optionally ICP correct, update grid."""
        self.state = "SCANNING"
        self.message = "Scanning..."

        try:
            scan = self.scanner.scan()
            pose = self.robot.get_pose()

            # ICP correction
            if self.use_icp and self.prev_scan_world is not None and len(scan) > 5:
                scan_world = self._scan_to_world(scan, pose)
                R, t, _, ok = icp(scan_world, self.prev_scan_world, max_distance=20)

                if ok:
                    corr_pos = R @ np.array([pose[0], pose[1]]) + t
                    corr_angle = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    dx = corr_pos[0] - pose[0]
                    dy = corr_pos[1] - pose[1]
                    self.robot.x = corr_pos[0]
                    self.robot.y = corr_pos[1]
                    self.robot._heading += corr_angle
                    pose = self.robot.get_pose()
                    self.icp_result = {
                        "status": "converged",
                        "dx": round(dx, 1),
                        "dy": round(dy, 1),
                        "dtheta": round(corr_angle, 1),
                    }
                    print(
                        f"ICP converged: dx={dx:.1f} dy={dy:.1f} dθ={corr_angle:.1f}°"
                    )
                else:
                    self.icp_result = {"status": "failed"}
                    print("ICP failed to converge")
            elif self.use_icp and self.prev_scan_world is None:
                self.icp_result = {"status": "first_scan"}

            # Update grid
            self.grid.update(scan, pose)
            self.pose = pose
            self.map_version += 1

            # Store for next ICP
            if self.use_icp:
                self.prev_scan_world = self._scan_to_world(scan, pose)

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Scan error: {e}"
            traceback.print_exc()

    def _scan_to_world(self, scan_sensor, pose):
        """Transform sensor-frame scan points to world frame."""
        x, y, heading_deg = pose
        theta = math.radians(heading_deg)
        c, s = math.cos(theta), math.sin(theta)
        R = np.array([[c, -s], [s, c]])
        scan_robot = scan_sensor + [TOF_OFFSET_X, TOF_OFFSET_Y]
        return (R @ scan_robot.T).T + np.array([x, y])
