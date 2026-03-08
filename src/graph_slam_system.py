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
    OPTIMIZE_EVERY_N = 5

    # Loop closure detection parameters
    LOOP_MIN_NODE_GAP = 10  # Minimum nodes between current and candidate
    LOOP_MAX_DIST_CM = 50.0  # Max distance (cm) to consider loop closure
    LOOP_MIN_MATCH_RATIO = 0.5  # ICP match ratio required for loop closure

    def __init__(self, use_icp=True):
        super().__init__(use_icp=use_icp)
        self.pose_graph = PoseGraph()
        self._prev_node_id: int | None = None
        self._prev_scan_world: np.ndarray | None = None
        self._nodes_since_optimize: int = 0
        self._num_optimizations: int = 0
        self._last_optimize_stats: dict | None = None

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
        """Initialize hardware and seed pose graph with initial scans.

        Same as parent but records each bootstrap scan as a graph node
        with an odometry edge between them.
        """
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

        Replaces SlamSystem's greedy ICP correction with graph-based approach:
        1. Take scan
        2. Add a node to the pose graph
        3. Add odometry edge from previous node
        4. Run scan-to-scan ICP → add ICP edge with confidence weight
        5. Check for loop closure candidates
        6. Every OPTIMIZE_EVERY_N nodes: optimize + rebuild map

        Args:
            force_update: If True, also update the working map immediately
                          (used for bootstrap scans before enough nodes exist).
        """
        self.state = "SCANNING"
        self.message = "Scanning..."

        try:
            t0 = time.monotonic()
            scan, free_rays = self.scanner.scan()
            t_scan = time.monotonic() - t0

            pose = self.robot.get_pose()

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
                        current_scan_world, self._prev_scan_world
                    )
                    if icp_result is not None:
                        icp_transform, icp_quality = icp_result
                        t_icp = icp_quality.get("time", 0)
                        icp_info = PoseGraph.compute_icp_info_matrix(
                            icp_quality["match_ratio"],
                            icp_quality["mean_error"],
                            icp_quality["converged"],
                        )
                        # Only add edge if info matrix is non-zero
                        # (compute_icp_info_matrix returns zeros for bad matches)
                        if np.any(icp_info > 0):
                            self.pose_graph.add_edge(
                                self._prev_node_id,
                                node_id,
                                icp_transform,
                                "icp_sequential",
                                icp_info,
                            )

                        self.icp_result = {
                            "status": "converged"
                            if icp_quality["converged"]
                            else "failed",
                            "match_ratio": round(icp_quality["match_ratio"] * 100),
                            "mean_error": round(icp_quality["mean_error"], 1),
                            "edge_added": bool(np.any(icp_info > 0)),
                        }
                    else:
                        self.icp_result = {
                            "status": "skipped",
                            "reason": "not enough points",
                        }

                # ── Loop closure detection ─────────────────────────
                n_loops = self._check_loop_closures(node_id, current_scan_world)
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
                self.pose_graph.rebuild_map(self.grid)
                t_opt = time.monotonic() - t1

                # Update our current pose to the latest optimized pose
                self.pose = self.pose_graph.get_corrected_pose(node_id)
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

            elif force_update:
                # During bootstrap, also update the map immediately
                self.grid.update(scan, pose, free_rays=free_rays)
                self.pose = pose
                self.map_version += 1

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
        current_scan_world: np.ndarray,
        previous_scan_world: np.ndarray,
    ) -> tuple[tuple[float, float, float], dict] | None:
        """Run ICP between two world-frame scans.

        Returns the relative transform FROM previous TO current, matching
        the edge convention (from_id=prev, to_id=current).

        icp(source, target) aligns source→target, so we call
        icp(previous, current) to get the prev→current transform.

        Returns:
            ((dx, dy, dtheta_deg), quality_dict) or None if too few points.
        """
        if len(current_scan_world) < 5 or len(previous_scan_world) < 5:
            return None

        t0 = time.monotonic()
        # icp(source, target) returns R,t that aligns source to target
        # We want prev→current, so source=previous, target=current
        R, t_vec, _, converged, info = icp(
            previous_scan_world, current_scan_world, max_distance=10
        )
        t_elapsed = time.monotonic() - t0

        # Extract rotation angle from R matrix
        dtheta_rad = math.atan2(R[1, 0], R[0, 0])
        dtheta_deg = math.degrees(dtheta_rad)

        # The ICP transform: t_vec is the translation, R is rotation
        dx, dy = float(t_vec[0]), float(t_vec[1])

        quality = {
            "match_ratio": info.get("match_ratio", 0),
            "mean_error": info.get("mean_error", 0),
            "converged": converged,
            "time": t_elapsed,
        }

        return (dx, dy, dtheta_deg), quality

    # ── Loop Closure ──────────────────────────────────────────────

    def _check_loop_closures(
        self, current_node_id: int, current_scan_world: np.ndarray
    ) -> int:
        """Check for loop closures and add edges if found.

        Args:
            current_node_id: ID of the node just added.
            current_scan_world: Current scan in world frame.

        Returns:
            Number of loop closure edges added.
        """
        current_pose = self.pose_graph.get_corrected_pose(current_node_id)
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

            # Run ICP: _run_scan_to_scan_icp(current, previous)
            # calls icp(previous, current) → returns transform prev→current
            # So this gives candidate→current transform
            result = self._run_scan_to_scan_icp(
                current_scan_world, candidate_scan_world
            )
            if result is None:
                continue

            icp_transform, icp_quality = result

            # Only accept high-quality loop closures
            if (
                icp_quality["converged"]
                and icp_quality["match_ratio"] >= self.LOOP_MIN_MATCH_RATIO
                and icp_quality["mean_error"] < 5.0
            ):
                # Use ICP info matrix but boost confidence for loop closures
                icp_info = PoseGraph.compute_icp_info_matrix(
                    icp_quality["match_ratio"],
                    icp_quality["mean_error"],
                    True,
                )
                # Boost: loop closures are high-value constraints
                icp_info *= 2.0

                # Edge direction: from=candidate → to=current
                # matches the ICP transform (candidate→current)
                self.pose_graph.add_edge(
                    candidate_id,
                    current_node_id,
                    icp_transform,
                    "loop_closure",
                    icp_info,
                )
                count += 1
                print(
                    f"Loop closure: node {current_node_id} ↔ node {candidate_id} "
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

    # ── Accessors for Web UI ──────────────────────────────────────

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
