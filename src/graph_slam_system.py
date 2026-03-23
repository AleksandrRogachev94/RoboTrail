"""Graph-based SLAM system.

Extends the existing SlamSystem with a pose graph backend. Instead of
greedily applying ICP corrections, this system:
  1. Records each scan as a node in a pose graph
  2. Adds odometry and ICP edges as constraints (with confidence weights)
  3. Periodically optimizes all poses jointly
  4. Rebuilds the map from corrected poses

The original SlamSystem is kept as a working fallback — this file
composes/wraps it rather than modifying it.

Usage:
    # In web_server.py, swap SlamSystem for GraphSlamSystem:
    # from graph_slam_system import GraphSlamSystem
    # slam = GraphSlamSystem()
"""

import math
import resource
import threading
import time
import traceback

import numpy as np

from frontier import find_frontiers, get_frontier_viz_data, select_goal
from icp import icp
from occupancy_grid import scan_to_world
from path_planner import plan_path
from pose_graph import PoseGraph
from slam_system import SlamSystem


class GraphSlamSystem(SlamSystem):
    """SLAM system with pose graph optimization.

    Inherits all hardware control, path planning, and exploration logic
    from SlamSystem. Overrides the scan-and-update pipeline to use
    graph-based constraint management instead of greedy ICP correction.
    """

    # How often to run graph optimization (every N nodes added)
    # Also optimizes immediately when a loop closure is detected.
    OPTIMIZE_EVERY_N = 5

    # Loop closure detection parameters
    LOOP_MIN_NODE_GAP = 5  # Minimum nodes between current and candidate
    LOOP_MAX_DIST_CM = 70.0  # Max distance (cm) to consider loop closure
    LOOP_MIN_MATCH_RATIO = 0.5  # ICP match ratio required for loop closure

    # ICP edge sanity thresholds: reject if ICP disagrees with odometry
    ICP_MAX_DIST_DIFF = 18.0  # cm — max displacement difference from odometry
    ICP_MAX_ANGLE_DIFF = 20.0  # degrees — max heading difference from odometry

    # Mid-turn scanning: if a turn exceeds this angle, scan mid-turn to
    # ensure ICP has enough overlap. With 180° FOV, a 90° turn leaves ~0%
    # overlap between consecutive scans. Scanning mid-turn guarantees ≥120° overlap.
    MAX_TURN_WITHOUT_SCAN = 60.0  # degrees

    def __init__(self, use_icp=True):
        super().__init__(use_icp=use_icp)
        self.pose_graph = PoseGraph()
        self._prev_node_id: int | None = None
        self._prev_scan_world: np.ndarray | None = None
        self._nodes_since_optimize: int = 0
        self._num_optimizations: int = 0
        self._last_optimize_stats: dict | None = None
        self._grid_lock = threading.Lock()
        self._loop_closure_pending: bool = False
        self._recently_closed_ids: set[int] = set()  # Skip redundant ICP retries

        # Graph data for web UI
        self.graph_info = {
            "num_nodes": 0,
            "num_edges": 0,
            "num_optimizations": 0,
            "last_optimize": None,
            "loop_closures": 0,
        }

    # ── Override: Initial Hardware Setup ───────────────────────────

    def _init_hardware(self):
        """Initialize hardware and seed pose graph with initial scans."""
        try:
            import lgpio

            from robot.drive_dc import RobotDC
            from scanner import Scanner

            print("Initializing hardware (graph SLAM)...")
            self.chip = lgpio.gpiochip_open(4)
            self.robot = RobotDC(self.chip)
            self.scanner = Scanner()

            # Initial scan at origin
            self._scan_and_update(force_update=True)

            # Three 90° turns + scans for full 360° coverage.
            for i in range(3):
                self.robot.imu.calibrate_gyro(samples=100)
                self.robot.turn(90)
                self.pose = self.robot.get_pose()
                self._scan_and_update(force_update=True)

            self.state = "IDLE"
            self.message = "Ready (graph SLAM)"
            print(
                f"Hardware ready. Graph: {self.pose_graph.num_nodes} nodes, "
                f"{self.pose_graph.num_edges} edges"
            )

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Init error: {e}"
            traceback.print_exc()

    # ── Override: Core Scan Pipeline ───────────────────────────────

    def _scan_and_update(self, force_update=False):
        """Scan, add to pose graph, and periodically optimize.

        Pipeline:
        1. Take scan → always add to map (so robot can navigate)
        2. Add node + odometry edge to pose graph
        3. Run scan-to-scan ICP → add ICP edge with confidence weight
        4. Check for loop closure candidates
        5. Every OPTIMIZE_EVERY_N nodes: optimize + rebuild map from scratch
        """
        self.state = "SCANNING"
        self.message = "Scanning..."

        try:
            t0 = time.monotonic()
            scan, free_rays = self.scanner.scan()
            t_scan = time.monotonic() - t0

            pose = self.robot.get_pose()

            # ── Always update the map with the new scan ────────────
            # This keeps the map fresh for navigation. Optimization
            # will rebuild from scratch with corrected poses later.
            with self._grid_lock:
                self.grid.update(scan, pose, free_rays=free_rays)
            self.pose = pose
            self.map_version += 1

            # ── Add node to pose graph ─────────────────────────────
            node_id = self.pose_graph.add_node(pose, scan, free_rays)
            current_scan_world, _ = scan_to_world(scan, pose)

            # ── Add odometry edge from previous node ───────────────
            t_icp = 0.0
            if self._prev_node_id is not None:
                prev_pose = self.pose_graph.get_corrected_pose(self._prev_node_id)
                odom_transform = self._compute_relative_transform(prev_pose, pose)

                # Odometry confidence: less confident for longer moves/turns
                dist = math.hypot(odom_transform[0], odom_transform[1])
                rot = abs(odom_transform[2])
                odom_info = PoseGraph.compute_odometry_info_matrix(dist, rot)
                self.pose_graph.add_edge(
                    self._prev_node_id, node_id, odom_transform, "odometry", odom_info
                )

                # ── Scan-to-scan ICP ───────────────────────────────
                if self.use_icp and self._prev_scan_world is not None:
                    icp_result = self._run_scan_to_scan_icp(
                        self._prev_scan_world, current_scan_world
                    )
                    if icp_result is not None:
                        R_icp, t_icp_vec, icp_quality = icp_result
                        t_icp = icp_quality.get("time", 0)

                        # Convert ICP world-frame result to local-frame transform
                        icp_transform = self._icp_to_local_transform(
                            R_icp, t_icp_vec, prev_pose, pose
                        )

                        # ── Sanity check: reject ICP if it disagrees with odometry ──
                        dx_diff = abs(icp_transform[0] - odom_transform[0])
                        dy_diff = abs(icp_transform[1] - odom_transform[1])
                        dist_diff = math.hypot(dx_diff, dy_diff)
                        angle_diff = abs(
                            (icp_transform[2] - odom_transform[2] + 180) % 360 - 180
                        )

                        icp_sane = (
                            dist_diff < self.ICP_MAX_DIST_DIFF
                            and angle_diff < self.ICP_MAX_ANGLE_DIFF
                        )

                        icp_info = PoseGraph.compute_icp_info_matrix(
                            icp_quality["match_ratio"],
                            icp_quality["mean_error"],
                            icp_quality["converged"],
                        )
                        # Only add edge if info matrix is non-zero AND sane
                        edge_added = False
                        if np.any(icp_info > 0) and icp_sane:
                            self.pose_graph.add_edge(
                                self._prev_node_id,
                                node_id,
                                icp_transform,
                                "icp_sequential",
                                icp_info,
                            )
                            edge_added = True
                        elif not icp_sane:
                            # ICP disagreed with odometry → this segment likely has
                            # undetected drift (featureless hallway). Penalize the
                            # odometry edge so loop closures can correct it later.
                            self.pose_graph.edges[-1].info_matrix *= 0.3
                            print(
                                f"ICP rejected (sanity): Δdist={dist_diff:.1f}cm "
                                f"Δangle={angle_diff:.1f}° vs odometry"
                            )

                        self.icp_result = {
                            "status": "converged"
                            if icp_quality["converged"]
                            else "failed",
                            "match_ratio": round(icp_quality["match_ratio"] * 100),
                            "mean_error": round(icp_quality["mean_error"], 1),
                            "dx": round(icp_transform[0], 1),
                            "dy": round(icp_transform[1], 1),
                            "dtheta": round(icp_transform[2], 1),
                            "edge_added": edge_added,
                        }
                    else:
                        self.icp_result = {
                            "status": "skipped",
                            "reason": "not enough points",
                        }

                # ── Loop closure detection ─────────────────────────
                n_loops = self._check_loop_closures(node_id, current_scan_world, pose)
                if n_loops > 0:
                    self.graph_info["loop_closures"] += n_loops
                    self._loop_closure_pending = True
                    print(f"Added {n_loops} loop closure edge(s)!")

            # ── Track state ────────────────────────────────────────
            self._prev_node_id = node_id
            self._prev_scan_world = current_scan_world
            self._nodes_since_optimize += 1

            # ── Optimization: on loop closure (immediate) or periodic ──
            t_opt = 0.0
            t_rebuild = 0.0
            did_optimize = False
            should_optimize = self.pose_graph.num_edges > 0 and (
                self._loop_closure_pending
                or self._nodes_since_optimize >= self.OPTIMIZE_EVERY_N
            )
            if should_optimize:
                try:
                    t1 = time.monotonic()
                    stats = self.pose_graph.optimize()
                    t_opt = time.monotonic() - t1

                    with self._grid_lock:
                        t_rebuild = self.pose_graph.rebuild_map(self.grid)

                    # Update our current pose to the latest optimized pose
                    corrected = self.pose_graph.get_corrected_pose(node_id)
                    # Normalize heading to [-180, 180]
                    norm_heading = (corrected[2] + 180) % 360 - 180
                    self.pose = (corrected[0], corrected[1], norm_heading)
                    self.robot.set_pose(*self.pose)

                    # Recompute _prev_scan_world using the corrected pose
                    # so the next scan-to-scan ICP uses the right reference
                    self._prev_scan_world, _ = scan_to_world(
                        self.pose_graph.get_node(node_id).scan_points,
                        self.pose,
                    )

                    self._nodes_since_optimize = 0
                    self._num_optimizations += 1
                    self._recently_closed_ids.clear()  # Poses changed; allow re-closure
                    self._last_optimize_stats = stats
                    self._loop_closure_pending = False
                    did_optimize = True

                    self.map_version += 1
                    print(
                        f"Graph optimized: cost {stats['initial_cost']:.1f} → {stats['final_cost']:.1f}, "
                        f"max_correction={stats['max_correction']:.1f}cm "
                        f"({t_opt:.2f}s opt + {t_rebuild:.2f}s rebuild)"
                    )
                except Exception as e:
                    print(f"⚠ Optimization failed: {e}")
                    traceback.print_exc()
                    # Reset counter so we retry later
                    self._nodes_since_optimize = 0
                    self._loop_closure_pending = False

            # ── Update graph info for UI ───────────────────────────
            self.graph_info.update(
                {
                    "num_nodes": self.pose_graph.num_nodes,
                    "num_edges": self.pose_graph.num_edges,
                    "num_optimizations": self._num_optimizations,
                    "last_optimize": self._last_optimize_stats,
                }
            )

            t_total = time.monotonic() - t0
            mem_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
            print(
                f"⏱ scan={t_scan:.2f}s icp={t_icp:.2f}s "
                f"opt={t_opt:.2f}s total={t_total:.2f}s "
                f"[nodes={self.pose_graph.num_nodes} edges={self.pose_graph.num_edges}"
                f"{' OPTIMIZED' if did_optimize else ''} mem={mem_mb:.0f}MB]"
            )

        except Exception as e:
            self.state = "ERROR"
            self.message = f"Scan error: {e}"
            traceback.print_exc()

    # ── Scan-to-Scan ICP ──────────────────────────────────────────

    def _run_scan_to_scan_icp(
        self,
        source_scan_world: np.ndarray,
        target_scan_world: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, dict] | None:
        """Run ICP to align source scan to target scan (both in world frame).

        icp(source, target) returns R, t such that R @ source + t ≈ target.

        Returns:
            (R, t_vec, quality_dict) or None if too few points.
            R: 2x2 rotation matrix
            t_vec: 2D translation vector (world frame)
        """
        if len(source_scan_world) < 5 or len(target_scan_world) < 5:
            return None

        t0 = time.monotonic()
        R, t_vec, _, converged, info = icp(
            source_scan_world, target_scan_world, max_distance=20
        )
        t_elapsed = time.monotonic() - t0

        quality = {
            "match_ratio": info.get("match_ratio", 0),
            "mean_error": info.get("mean_error", 0),
            "converged": converged,
            "time": t_elapsed,
        }

        return R, t_vec, quality

    def _icp_to_local_transform(
        self,
        R_icp: np.ndarray,
        t_icp: np.ndarray,
        source_pose: tuple[float, float, float],
        target_pose: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Convert ICP world-frame result to a local-frame relative transform.

        ICP says: R @ source_pts + t ≈ target_pts. This means the source
        scan should be corrected by (R, t) to match the target. We compute
        the corrected source pose, then find the relative transform from
        corrected-source to target in the corrected-source's local frame.

        Args:
            R_icp: 2x2 rotation from ICP.
            t_icp: 2D translation from ICP (world frame).
            source_pose: (x, y, heading_deg) of the source node.
            target_pose: (x, y, heading_deg) of the target node.

        Returns:
            (dx_local, dy_local, dtheta_deg) in corrected-source's local frame.
        """
        # Correct the source node's position using ICP
        src_xy = np.array([source_pose[0], source_pose[1]])
        corrected_xy = R_icp @ src_xy + t_icp
        dtheta_deg = math.degrees(math.atan2(R_icp[1, 0], R_icp[0, 0]))
        corrected_src_pose = (
            float(corrected_xy[0]),
            float(corrected_xy[1]),
            source_pose[2] + dtheta_deg,
        )

        return self._compute_relative_transform(corrected_src_pose, target_pose)

    # ── Loop Closure ──────────────────────────────────────────────

    def _check_loop_closures(
        self,
        current_node_id: int,
        current_scan_world: np.ndarray,
        current_pose: tuple[float, float, float],
    ) -> int:
        """Check for loop closures and add edges if found.

        Args:
            current_node_id: ID of the node just added.
            current_scan_world: Current scan in world frame.
            current_pose: Current robot pose.

        Returns:
            Number of loop closure edges added.
        """
        candidates = self.pose_graph.find_loop_candidates(
            current_pose,
            min_node_gap=self.LOOP_MIN_NODE_GAP,
            max_distance_cm=self.LOOP_MAX_DIST_CM,
        )

        if not candidates:
            return 0

        count = 0
        for candidate_id in candidates[:3]:  # Try top 3 closest
            if candidate_id in self._recently_closed_ids:
                continue
            candidate_node = self.pose_graph.get_node(candidate_id)
            candidate_pose = candidate_node.pose

            # Transform candidate's sensor-frame scan to world frame
            candidate_scan_world, _ = scan_to_world(
                candidate_node.scan_points, candidate_pose
            )

            # ICP with wider max_distance for loop closure (more drift expected)
            if len(candidate_scan_world) < 5 or len(current_scan_world) < 5:
                continue
            R_icp, t_icp, _, converged, info = icp(
                candidate_scan_world, current_scan_world, max_distance=30
            )
            icp_quality = {
                "match_ratio": info.get("match_ratio", 0),
                "mean_error": info.get("mean_error", 0),
                "converged": converged,
            }
            result = (
                (R_icp, t_icp, icp_quality)
                if converged or info.get("match_ratio", 0) > 0
                else None
            )
            if result is None:
                print(f"  Loop candidate {candidate_id}: ICP failed to converge")
                continue

            R_icp, t_icp, icp_quality = result

            # Accept loop closure based on match quality only
            # (converged flag just means ICP hit max iterations — irrelevant)
            if not (
                icp_quality["match_ratio"] >= self.LOOP_MIN_MATCH_RATIO
                and icp_quality["mean_error"] < 8.0
            ):
                print(
                    f"  Loop candidate {candidate_id}: rejected "
                    f"(match={icp_quality['match_ratio'] * 100:.0f}% "
                    f"err={icp_quality['mean_error']:.1f}cm "
                    f"converged={icp_quality['converged']})"
                )
                continue

            # Convert ICP result to local-frame transform
            icp_transform = self._icp_to_local_transform(
                R_icp, t_icp, candidate_pose, current_pose
            )

            icp_info = PoseGraph.compute_icp_info_matrix(
                icp_quality["match_ratio"],
                icp_quality["mean_error"],
                True,
            )
            # Boost: loop closures are high-value constraints.
            # Needs to be strong enough to overpower chains of
            # uncorrected odometry edges in featureless areas.
            icp_info *= 5.0

            # Edge: from=candidate → to=current
            self.pose_graph.add_edge(
                candidate_id,
                current_node_id,
                icp_transform,
                "loop_closure",
                icp_info,
            )
            count += 1
            self._recently_closed_ids.add(candidate_id)
            print(
                f"Loop closure: node {candidate_id} → node {current_node_id} "
                f"(match={icp_quality['match_ratio'] * 100:.0f}% "
                f"err={icp_quality['mean_error']:.1f}cm)"
            )
            break  # One loop closure per scan is enough

        return count

    # ── Odometry Transform ────────────────────────────────────────

    @staticmethod
    def _compute_relative_transform(
        pose_from: tuple[float, float, float],
        pose_to: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Compute relative transform between two poses in source's local frame.

        Args:
            pose_from: Source pose (x, y, heading_deg).
            pose_to: Target pose (x, y, heading_deg).

        Returns:
            (dx_local, dy_local, dtheta_deg) in source's local frame.
        """
        dx_global = pose_to[0] - pose_from[0]
        dy_global = pose_to[1] - pose_from[1]
        theta = math.radians(pose_from[2])

        # Rotate global displacement into source's local frame
        dx_local = math.cos(theta) * dx_global + math.sin(theta) * dy_global
        dy_local = -math.sin(theta) * dx_global + math.cos(theta) * dy_global

        # Heading difference, normalized to [-180, 180]
        dtheta = (pose_to[2] - pose_from[2] + 180) % 360 - 180

        return (dx_local, dy_local, dtheta)

    # ── Override: Split Large Turns ───────────────────────────────────

    def _drive_one_step(self, target: tuple):
        """Plan path to target, drive to the first waypoint, and scan.

        Overrides parent to split large turns: if the heading change to the
        next waypoint exceeds MAX_TURN_WITHOUT_SCAN (60°), we turn first,
        take a mid-turn scan (giving ICP overlap), then drive forward.
        """
        tx, ty = target
        from path_planner import plan_path

        start_xy = (self.pose[0], self.pose[1])
        waypoints = plan_path(self.grid, start_xy, (tx, ty))

        if waypoints is None or len(waypoints) < 2:
            print(f"No path to ({tx:.0f}, {ty:.0f})")
            return

        self.planned_waypoints = [(round(x, 1), round(y, 1)) for x, y in waypoints]
        next_x, next_y = waypoints[1]

        try:
            self.robot.imu.calibrate_gyro(samples=100)
            self.state = "MOVING"
            self.message = f"Moving to ({next_x:.0f}, {next_y:.0f})..."
            self.robot.history = []

            # Compute heading change needed
            dx = next_x - self.pose[0]
            dy = next_y - self.pose[1]
            dist = math.hypot(dx, dy)
            target_heading = math.degrees(math.atan2(dy, dx))
            heading_error = (target_heading - self.pose[2] + 180) % 360 - 180

            if abs(heading_error) > self.MAX_TURN_WITHOUT_SCAN and dist > 1.0:
                # Large turn: split into turn → scan → forward
                self.robot.turn(heading_error)
                self.pose = self.robot.get_pose()
                self.robot.set_pose(
                    self.pose[0],
                    self.pose[1],
                    (self.pose[2] + 180) % 360 - 180,
                )
                self.pose = self.robot.get_pose()
                # Mid-turn scan: gives ICP overlap after the rotation
                self._scan_and_update()
                # Now drive forward (heading is already set)
                self.robot.forward(dist)
            else:
                # Small turn: move_to handles it atomically
                self.robot.move_to(next_x, next_y)

            self.pose = self.robot.get_pose()
            self.robot.set_pose(
                self.pose[0],
                self.pose[1],
                (self.pose[2] + 180) % 360 - 180,
            )
            self.pose = self.robot.get_pose()
            self.path_history.append((self.pose[0], self.pose[1]))

            if self.robot.history:
                h = self.robot.history
                step = max(1, len(h) // 100)
                self.pid_summary = {
                    "t": [round(s["t"], 3) for s in h[::step]],
                    "left_pwm": [round(s["left_pwm"]) for s in h[::step]],
                    "right_pwm": [round(s["right_pwm"]) for s in h[::step]],
                    "heading_error": [round(s["heading_error"], 1) for s in h[::step]],
                }
        except Exception as e:
            self.state = "ERROR"
            self.message = f"Move failed: {e}"
            traceback.print_exc()
            return

        # Scan after arriving at waypoint
        odom_pos = (self.pose[0], self.pose[1])
        self._scan_and_update()

        corrected_pos = (self.pose[0], self.pose[1])
        if self.icp_result and self.icp_result.get("status") == "converged":
            if self.path_history:
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

        self.planned_waypoints = []

    # ── On-the-Way Loop Closure ─────────────────────────────────────

    def _explore_loop(self):
        """Explore frontiers with opportunistic on-the-way loop closure.

        Before driving to a frontier, checks if the planned path passes
        near an old pose graph node. If so, detours to that node for a
        scan (triggering loop closure detection), then resumes frontier
        exploration on the next call.

        When exploration finishes (no frontiers), runs a post-exploration
        sweep to close any remaining loops.
        """
        self.state = "EXPLORING"
        self.message = "Detecting frontiers..."

        t0 = time.monotonic()
        clusters = find_frontiers(self.grid)
        t_frontier = time.monotonic() - t0
        self.frontier_data = get_frontier_viz_data(self.grid, clusters, self.pose)

        if not clusters:
            if self.map_version <= 2:
                print("No frontiers yet (early map) — driving forward")
                self._drive_forward_safely(self.BOOTSTRAP_DRIVE_CM)
                return
            print("Exploration complete — running post-exploration loop closures")
            self._post_exploration_loop_closure()
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

        t1 = time.monotonic()
        goal = select_goal(self.grid, clusters, self.pose, min_distance_cm=25.0)
        t_goal = time.monotonic() - t1
        print(f"⏱ frontiers={t_frontier:.2f}s goal_select={t_goal:.2f}s")

        if goal is None:
            if self.map_version <= 2:
                print("No reachable frontiers (early map) — driving forward")
                self._drive_forward_safely(self.BOOTSTRAP_DRIVE_CM)
                return
            print("No reachable frontiers — running post-exploration loop closures")
            self._post_exploration_loop_closure()
            self._exploring = False
            self.explore_goal = None
            self.state = "IDLE"
            self.message = "No reachable frontiers — exploration stopped."
            print("No reachable frontiers.")
            return

        gx, gy = goal
        dist = math.hypot(gx - self.pose[0], gy - self.pose[1])
        print(f"Explore goal: ({gx:.0f}, {gy:.0f}), dist={dist:.0f}cm")
        self.explore_goal = goal
        self.message = f"Exploring → ({gx:.0f}, {gy:.0f})"

        # Drive one step toward frontier.
        # Passive _check_loop_closures fires after each scan along the way,
        # handling loop closures automatically when near old nodes.
        self._drive_one_step(goal)
        self.explore_goal = None

    def _post_exploration_loop_closure(self, max_targets: int = 3):
        """After exploration finishes, visit high-value old nodes for loop closure.

        Targets are scored by graph_gap / spatial_distance — nodes that are
        far in the graph but near in space offer the most drift correction.
        """
        current_id = self.pose_graph.get_latest_node_id()
        if current_id is None or current_id < 10:
            return

        rx, ry = self.pose[0], self.pose[1]
        targets = []

        for nid, node in self.pose_graph.nodes.items():
            graph_gap = current_id - nid
            if graph_gap < self.LOOP_MIN_NODE_GAP:
                continue
            spatial_dist = math.hypot(node.pose[0] - rx, node.pose[1] - ry)
            if spatial_dist < 30:
                continue
            score = graph_gap / (spatial_dist + 1.0)
            targets.append((score, nid, node))

        targets.sort(reverse=True)

        for score, nid, node in targets[:max_targets]:
            waypoints = plan_path(
                self.grid,
                (self.pose[0], self.pose[1]),
                (node.pose[0], node.pose[1]),
            )
            if waypoints is None:
                print(f"🔄 Post-exploration: no path to node {nid}, skipping")
                continue

            print(
                f"🔄 Post-exploration loop closure: driving to node {nid} "
                f"(score={score:.2f})"
            )
            self.state = "EXPLORING"
            self.message = f"Loop closure sweep → node {nid}"
            self._move_scan_update((node.pose[0], node.pose[1]))

    # ── Accessors for Web UI ──────────────────────────────────────

    def get_map_data(self):
        """Thread-safe version of parent's get_map_data."""
        with self._grid_lock:
            return super().get_map_data()

    def get_graph_data(self) -> dict:
        """Return pose graph data for visualization in the web UI."""
        nodes = []
        for nid in sorted(self.pose_graph.nodes.keys()):
            node = self.pose_graph.nodes[nid]
            nodes.append(
                {
                    "id": nid,
                    "x": round(node.pose[0], 1),
                    "y": round(node.pose[1], 1),
                    "heading": round(node.pose[2], 1),
                }
            )

        edges = []
        for edge in self.pose_graph.edges:
            edges.append(
                {
                    "from": edge.from_id,
                    "to": edge.to_id,
                    "type": edge.edge_type,
                }
            )

        return {
            "nodes": nodes,
            "edges": edges,
            **self.graph_info,
        }
