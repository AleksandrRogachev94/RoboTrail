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
import threading
import time
import traceback

import numpy as np

from icp import icp
from occupancy_grid import scan_to_world
from pose_graph import PoseGraph
from slam_system import SlamSystem


class GraphSlamSystem(SlamSystem):
    """SLAM system with pose graph optimization.

    Inherits all hardware control, path planning, and exploration logic
    from SlamSystem. Overrides the scan-and-update pipeline to use
    graph-based constraint management instead of greedy ICP correction.
    """

    # How often to run graph optimization (every N nodes added)
    OPTIMIZE_EVERY_N = 1

    # Loop closure detection parameters
    LOOP_MIN_NODE_GAP = 10  # Minimum nodes between current and candidate
    LOOP_MAX_DIST_CM = 50.0  # Max distance (cm) to consider loop closure
    LOOP_MIN_MATCH_RATIO = 0.5  # ICP match ratio required for loop closure

    # ICP edge sanity thresholds: reject if ICP disagrees with odometry
    ICP_MAX_DIST_DIFF = 15.0  # cm — max displacement difference from odometry
    ICP_MAX_ANGLE_DIFF = 15.0  # degrees — max heading difference from odometry

    # Active loop closure: every N exploration steps, drive to an old node
    LOOP_CLOSURE_INTERVAL = 8

    def __init__(self, use_icp=True):
        super().__init__(use_icp=use_icp)
        self.pose_graph = PoseGraph()
        self._prev_node_id: int | None = None
        self._prev_scan_world: np.ndarray | None = None
        self._nodes_since_optimize: int = 0
        self._num_optimizations: int = 0
        self._last_optimize_stats: dict | None = None
        self._grid_lock = threading.Lock()
        self._explore_steps: int = 0

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
                    print(f"Added {n_loops} loop closure edge(s)!")

            # ── Track state ────────────────────────────────────────
            self._prev_node_id = node_id
            self._prev_scan_world = current_scan_world
            self._nodes_since_optimize += 1

            # ── Periodic optimization ──────────────────────────────
            t_opt = 0.0
            did_optimize = False
            if (
                self._nodes_since_optimize >= self.OPTIMIZE_EVERY_N
                and self.pose_graph.num_edges > 0
            ):
                t1 = time.monotonic()
                stats = self.pose_graph.optimize()
                with self._grid_lock:
                    self.pose_graph.rebuild_map(self.grid)
                t_opt = time.monotonic() - t1

                # Update our current pose to the latest optimized pose
                corrected = self.pose_graph.get_corrected_pose(node_id)
                # Normalize heading to [-180, 180]
                norm_heading = (corrected[2] + 180) % 360 - 180
                self.pose = (corrected[0], corrected[1], norm_heading)
                self.robot.set_pose(*self.pose)

                self._nodes_since_optimize = 0
                self._num_optimizations += 1
                self._last_optimize_stats = stats
                did_optimize = True

                self.map_version += 1
                print(
                    f"Graph optimized: cost {stats['initial_cost']:.1f} → {stats['final_cost']:.1f}, "
                    f"max_correction={stats['max_correction']:.1f}cm ({t_opt:.2f}s)"
                )

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
            print(
                f"⏱ scan={t_scan:.2f}s icp={t_icp:.2f}s "
                f"opt={t_opt:.2f}s total={t_total:.2f}s "
                f"[nodes={self.pose_graph.num_nodes} edges={self.pose_graph.num_edges}"
                f"{' OPTIMIZED' if did_optimize else ''}]"
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
            source_scan_world, target_scan_world, max_distance=10
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
            candidate_node = self.pose_graph.get_node(candidate_id)
            candidate_pose = candidate_node.pose

            # Transform candidate's sensor-frame scan to world frame
            candidate_scan_world, _ = scan_to_world(
                candidate_node.scan_points, candidate_pose
            )

            # ICP: align candidate → current (source=candidate, target=current)
            result = self._run_scan_to_scan_icp(
                candidate_scan_world, current_scan_world
            )
            if result is None:
                continue

            R_icp, t_icp, icp_quality = result

            # Only accept high-quality loop closures
            if (
                icp_quality["converged"]
                and icp_quality["match_ratio"] >= self.LOOP_MIN_MATCH_RATIO
                and icp_quality["mean_error"] < 5.0
            ):
                # Convert ICP result to local-frame transform
                icp_transform = self._icp_to_local_transform(
                    R_icp, t_icp, candidate_pose, current_pose
                )

                icp_info = PoseGraph.compute_icp_info_matrix(
                    icp_quality["match_ratio"],
                    icp_quality["mean_error"],
                    True,
                )
                # Boost: loop closures are high-value constraints
                icp_info *= 2.0

                # Edge: from=candidate → to=current
                self.pose_graph.add_edge(
                    candidate_id,
                    current_node_id,
                    icp_transform,
                    "loop_closure",
                    icp_info,
                )
                count += 1
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

    # ── Active Loop Closure ───────────────────────────────────────

    def _explore_loop(self):
        """Override: periodically seek loop closures during exploration."""
        self._explore_steps += 1

        # Every N steps, try active loop closure instead of frontier
        if (
            self._explore_steps % self.LOOP_CLOSURE_INTERVAL == 0
            and self.pose_graph.num_nodes >= self.LOOP_MIN_NODE_GAP + 2
        ):
            target = self._find_loop_closure_target()
            if target is not None:
                target_pos, target_id = target
                self.state = "EXPLORING"
                self.message = f"Loop closure → node {target_id}"
                print(
                    f"Active loop closure: driving to node {target_id} "
                    f"at ({target_pos[0]:.0f}, {target_pos[1]:.0f})"
                )
                self._move_scan_update(target_pos[:2])
                return

        # Otherwise, normal frontier exploration
        super()._explore_loop()

    def _find_loop_closure_target(self) -> tuple[tuple, int] | None:
        """Find the best old node to revisit for loop closure.

        Picks the spatially closest node that is far enough in the graph
        (min_node_gap) and reachable via the traversability grid.

        Returns:
            ((x, y, heading), node_id) or None.
        """
        current_id = self.pose_graph.get_latest_node_id()
        if current_id is None:
            return None

        rx, ry = self.pose[0], self.pose[1]
        best = None
        best_dist = float("inf")

        for nid, node in self.pose_graph.nodes.items():
            # Must be old enough
            if current_id - nid < self.LOOP_MIN_NODE_GAP:
                continue
            dist = math.hypot(node.pose[0] - rx, node.pose[1] - ry)
            # Within reasonable range but not too close
            if 30 < dist < self.LOOP_MAX_DIST_CM and dist < best_dist:
                best_dist = dist
                best = (node.pose, nid)

        return best

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
