"""Pose graph for graph-based SLAM.

Maintains a graph of robot poses (nodes) connected by constraints (edges).
Edges come from odometry, scan-to-scan ICP, and loop closures. The optimizer
finds the set of poses that best satisfies all constraints simultaneously.

Uses scipy.optimize.least_squares with Huber loss for robust optimization
(automatically down-weights outlier edges from bad ICP matches).

Usage:
    graph = PoseGraph()
    n0 = graph.add_node(pose=(0, 0, 0), scan_points=scan0, free_rays=rays0)
    n1 = graph.add_node(pose=(10, 0, 0), scan_points=scan1, free_rays=rays1)
    graph.add_edge(n0, n1, transform=(10, 0, 0), edge_type="odometry",
                   info_matrix=np.eye(3))
    graph.optimize()
    graph.rebuild_map(occupancy_grid)
"""

import math
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from occupancy_grid import OccupancyGrid

# ── Data Structures ───────────────────────────────────────────────────


@dataclass
class PoseNode:
    """A node in the pose graph representing one scan position.

    Attributes:
        id: Unique node identifier (auto-assigned by PoseGraph).
        pose: Robot pose (x_cm, y_cm, heading_deg) when scan was taken.
        scan_points: (N, 2) array of scan hits in sensor frame.
                     Kept in sensor frame so we can replay at corrected poses.
        free_rays: (M, 2) array of beyond-range ray endpoints in sensor frame.
                   Needed for marking free space during map rebuild.
        timestamp: Time when this node was created (monotonic clock).
    """

    id: int
    pose: tuple[float, float, float]  # (x, y, heading_deg)
    scan_points: np.ndarray  # sensor-frame scan hits
    free_rays: np.ndarray | None  # sensor-frame free ray endpoints
    timestamp: float = field(default_factory=time.monotonic)


@dataclass
class PoseEdge:
    """A constraint between two nodes in the pose graph.

    Attributes:
        from_id: Source node ID.
        to_id: Target node ID.
        transform: Measured relative transform (dx, dy, dtheta_deg)
                   from source to target in source's local frame.
        edge_type: One of "odometry", "icp_sequential", "loop_closure".
        info_matrix: 3×3 information matrix (inverse covariance).
                     Higher values = more confidence in this constraint.
    """

    from_id: int
    to_id: int
    transform: tuple[float, float, float]  # (dx, dy, dtheta_deg)
    edge_type: str  # "odometry", "icp_sequential", "loop_closure"
    info_matrix: np.ndarray  # 3×3


# ── Pose Graph ─────────────────────────────────────────────────────────


class PoseGraph:
    """Graph-based SLAM backend.

    Collects pose nodes and constraint edges, then optimizes all poses
    jointly using least squares to find the globally consistent trajectory.
    """

    def __init__(self):
        self.nodes: dict[int, PoseNode] = {}
        self.edges: list[PoseEdge] = []
        self._next_id: int = 0

    # ── Node Management ────────────────────────────────────────────

    def add_node(
        self,
        pose: tuple[float, float, float],
        scan_points: np.ndarray,
        free_rays: np.ndarray | None = None,
    ) -> int:
        """Add a new pose node to the graph.

        Args:
            pose: Robot pose (x, y, heading_deg) at scan time.
            scan_points: (N, 2) scan hits in sensor frame.
            free_rays: (M, 2) free ray endpoints in sensor frame.

        Returns:
            The new node's ID.
        """
        node = PoseNode(
            id=self._next_id, pose=pose, scan_points=scan_points, free_rays=free_rays
        )
        self.nodes[self._next_id] = node
        self._next_id += 1
        return node.id

    def get_node(self, node_id: int) -> PoseNode:
        """Retrieve a node by ID.

        Raises:
            KeyError: If node_id doesn't exist.
        """
        if node_id not in self.nodes:
            raise KeyError(f"node_id {node_id} does not exist")
        return self.nodes[node_id]

    @property
    def num_nodes(self) -> int:
        """Number of nodes in the graph."""
        return len(self.nodes)

    @property
    def num_edges(self) -> int:
        """Number of edges in the graph."""
        return len(self.edges)

    # ── Edge Management ────────────────────────────────────────────

    def add_edge(
        self,
        from_id: int,
        to_id: int,
        transform: tuple[float, float, float],
        edge_type: str,
        info_matrix: np.ndarray,
    ) -> None:
        """Add a constraint edge between two nodes.

        Args:
            from_id: Source node ID.
            to_id: Target node ID.
            transform: Relative transform (dx, dy, dtheta_deg) measured
                       in the source node's local frame.
            edge_type: "odometry", "icp_sequential", or "loop_closure".
            info_matrix: 3×3 information matrix (inverse covariance).

        Raises:
            KeyError: If from_id or to_id doesn't exist.
            ValueError: If edge_type is invalid.
        """
        self.get_node(from_id)  # validate
        self.get_node(to_id)  # validate
        edge = PoseEdge(
            from_id=from_id,
            to_id=to_id,
            transform=transform,
            edge_type=edge_type,
            info_matrix=info_matrix,
        )
        self.edges.append(edge)

    # ── Information Matrix Construction ────────────────────────────

    @staticmethod
    def compute_odometry_info_matrix(
        distance_cm: float,
        rotation_deg: float,
    ) -> np.ndarray:
        """Build information matrix for an odometry edge.

        Odometry uncertainty grows with distance traveled and rotation.
        Longer moves and bigger turns → less confident → smaller info values.

        Args:
            distance_cm: Distance traveled between poses.
            rotation_deg: Rotation between poses (absolute value).

        Returns:
            3×3 information matrix (diagonal).

        Hints for implementation:
            - Translation sigma ≈ proportional to distance (e.g., 5% of distance)
            - Rotation sigma ≈ proportional to rotation (e.g., 2-5% of rotation)
            - Info = diag(1/σ²_x, 1/σ²_y, 1/σ²_θ)
            - Clamp minimum sigma to prevent division by zero
        """
        sigma_x = max(0.05 * distance_cm, 0.1)
        sigma_y = max(0.05 * distance_cm, 0.1)
        # Convert to radians — the residual function works in radians,
        # so the info matrix theta component must match
        sigma_theta = max(0.02 * math.radians(abs(rotation_deg)), 0.01)
        return np.diag([1 / sigma_x**2, 1 / sigma_y**2, 1 / sigma_theta**2])

    @staticmethod
    def compute_icp_info_matrix(
        match_ratio: float,
        mean_error: float,
        converged: bool,
    ) -> np.ndarray:
        """Build information matrix for an ICP edge based on match quality.

        ICP quality metrics map to confidence:
          - High match_ratio + low mean_error → tight (high confidence)
          - Low match_ratio or high error → loose (low confidence)

        Args:
            match_ratio: Fraction of scan points that found matches [0, 1].
            mean_error: Average distance of matched point pairs (cm).
            converged: Whether ICP converged within max iterations.

        Returns:
            3×3 information matrix (diagonal), or zeros if confidence
            is too low to use this edge.

        Hints for implementation:
            - Base sigma from mean_error (e.g., σ_trans = mean_error * scale)
            - Scale confidence by match_ratio (low match = high sigma)
            - If not converged, return very loose or zero matrix
            - Consider minimum match_ratio threshold below which
              you don't add the edge at all (return zeros)
        """
        # Reject: unconverged or too few matches to be meaningful
        if not converged or match_ratio < 0.3:
            return np.zeros((3, 3))

        # Base translation sigma from mean error (cm)
        # Good match (2cm error, 90% ratio) → σ ≈ 0.44cm → info ≈ 5.1
        # Okay match (5cm error, 50% ratio) → σ ≈ 2.0cm  → info ≈ 0.25
        sigma_trans = max(mean_error / (match_ratio * 5.0), 0.1)

        # Angular uncertainty: harder to estimate from ICP metrics.
        # Scale similarly but with a floor — ICP angular accuracy is
        # roughly proportional to translational accuracy for our scan geometry.
        sigma_theta_deg = max(mean_error / (match_ratio * 2.0), 0.5)
        sigma_theta_rad = math.radians(sigma_theta_deg)

        return np.diag(
            [
                1 / sigma_trans**2,
                1 / sigma_trans**2,
                1 / sigma_theta_rad**2,
            ]
        )

    # ── Optimization ───────────────────────────────────────────────

    def optimize(self) -> dict:
        """Optimize all poses to minimize total edge constraint error.

        Uses scipy.optimize.least_squares with Huber loss for robustness
        against outlier edges (bad ICP matches get auto-down-weighted).

        The first node is held fixed (anchor) to prevent the whole graph
        from drifting.

        Returns:
            dict with optimization stats:
                - "num_nodes": int
                - "num_edges": int
                - "initial_cost": float
                - "final_cost": float
                - "max_correction": float (largest pose change in cm)
        """
        if len(self.nodes) < 2 or len(self.edges) == 0:
            return {
                "num_nodes": len(self.nodes),
                "num_edges": 0,
                "initial_cost": 0,
                "final_cost": 0,
                "max_correction": 0,
            }

        # Step 1: Pack all node poses into a flat vector.
        # Order is [x0, y0, θ0, x1, y1, θ1, ...] with θ in RADIANS.
        # We need a consistent ordering, so use sorted node IDs.
        node_ids = sorted(self.nodes.keys())
        id_to_index = {nid: idx for idx, nid in enumerate(node_ids)}

        n = len(node_ids)
        x0 = np.zeros(3 * n)
        for nid in node_ids:
            idx = id_to_index[nid]
            node = self.nodes[nid]
            x0[3 * idx] = node.pose[0]
            x0[3 * idx + 1] = node.pose[1]
            x0[3 * idx + 2] = math.radians(node.pose[2])

        # Step 2: Compute initial cost (for stats).
        initial_residual = self._residual(x0)
        initial_cost = float(np.sum(initial_residual**2))

        # Step 3: Run scipy optimizer.
        # loss='huber' automatically down-weights large residuals (outliers).
        # This means a single bad ICP edge won't corrupt the whole graph.
        result = least_squares(self._residual, x0, loss="huber")

        # Step 4: Unpack optimized poses back into nodes.
        # Track the largest correction for diagnostics.
        # Cap corrections to prevent violent single-step jumps that create
        # ghosting when the map is rebuilt. Large corrections will converge
        # over multiple optimization calls.
        MAX_CORRECTION_CM = 30.0
        max_correction = 0.0
        for nid in node_ids:
            idx = id_to_index[nid]
            new_x = result.x[3 * idx]
            new_y = result.x[3 * idx + 1]
            new_th_rad = result.x[3 * idx + 2]
            new_th_deg = math.degrees(new_th_rad)

            old_pose = self.nodes[nid].pose
            correction = math.hypot(new_x - old_pose[0], new_y - old_pose[1])
            max_correction = max(max_correction, correction)

            # Cap: if correction is too large, blend toward optimized pose
            if correction > MAX_CORRECTION_CM and correction > 0:
                blend = MAX_CORRECTION_CM / correction
                new_x = old_pose[0] + blend * (new_x - old_pose[0])
                new_y = old_pose[1] + blend * (new_y - old_pose[1])
                # Blend angle too (in radians for proper interpolation)
                old_th_rad = math.radians(old_pose[2])
                angle_diff = (new_th_rad - old_th_rad + math.pi) % (
                    2 * math.pi
                ) - math.pi
                new_th_rad = old_th_rad + blend * angle_diff
                new_th_deg = math.degrees(new_th_rad)

            # Update the node's pose with the (possibly capped) values
            # Normalize heading to [-180, 180] to prevent accumulation
            new_th_deg = (new_th_deg + 180) % 360 - 180
            self.nodes[nid].pose = (new_x, new_y, new_th_deg)

        final_cost = float(result.cost)

        return {
            "num_nodes": n,
            "num_edges": len(self.edges),
            "initial_cost": round(initial_cost, 2),
            "final_cost": round(final_cost, 2),
            "max_correction": round(max_correction, 2),
        }

    def _residual(self, poses_flat: np.ndarray) -> np.ndarray:
        """Compute residual vector for all edges.

        For each edge, the residual is:
            r = sqrt_info @ (predicted_transform - measured_transform)

        where predicted_transform is computed from the current node poses
        and measured_transform is what the edge recorded.

        Args:
            poses_flat: 1D array [x0, y0, θ0, x1, y1, θ1, ...] in RADIANS.
                        Node i's pose lives at indices [3*i, 3*i+1, 3*i+2].

        Returns:
            1D array of all residuals stacked together.
            - 3 values per edge (dx_err, dy_err, dθ_err, each weighted)
            - Plus 3 values for the anchor prior on node 0
        """
        residuals = []

        # ── Anchor: pin node 0 to its initial position ─────────────
        # Without this, the optimizer can slide the entire graph around
        # (all edges are relative, so shifting everything preserves them).
        # We add a strong "virtual edge" that says node 0 should stay put.
        ANCHOR_WEIGHT = 1000.0
        x0, y0, th0 = poses_flat[0], poses_flat[1], poses_flat[2]
        node0 = self.nodes[0]
        anchor_x = node0.pose[0]
        anchor_y = node0.pose[1]
        anchor_th = math.radians(node0.pose[2])
        residuals.extend(
            [
                ANCHOR_WEIGHT * (x0 - anchor_x),
                ANCHOR_WEIGHT * (y0 - anchor_y),
                ANCHOR_WEIGHT * self._normalize_angle(th0 - anchor_th),
            ]
        )

        # ── Per-edge residuals ─────────────────────────────────────
        for edge in self.edges:
            # Step 1: Extract poses from flat vector
            i = edge.from_id
            j = edge.to_id
            xi, yi, thi = (
                poses_flat[3 * i],
                poses_flat[3 * i + 1],
                poses_flat[3 * i + 2],
            )
            xj, yj, thj = (
                poses_flat[3 * j],
                poses_flat[3 * j + 1],
                poses_flat[3 * j + 2],
            )

            # Step 2: Predicted relative transform in node i's local frame
            #
            # The global displacement (xj-xi, yj-yi) needs to be rotated
            # into node i's local frame. Think of it as: "standing at node i,
            # facing direction θ_i, how far forward/left is node j?"
            #
            #   local_x =  cos(θ_i) * dx_global + sin(θ_i) * dy_global
            #   local_y = -sin(θ_i) * dx_global + cos(θ_i) * dy_global
            #
            dx_global = xj - xi
            dy_global = yj - yi
            cos_i = math.cos(thi)
            sin_i = math.sin(thi)
            dx_pred = cos_i * dx_global + sin_i * dy_global
            dy_pred = -sin_i * dx_global + cos_i * dy_global
            dth_pred = thj - thi

            # Step 3: Measured transform (edge stores degrees, convert to rad)
            dx_meas, dy_meas, dth_meas_deg = edge.transform
            dth_meas = math.radians(dth_meas_deg)

            # Step 4: Error = predicted - measured
            # Critical: normalize angle error to [-π, π] so the optimizer
            # doesn't think 1° and 359° are 358° apart
            error = np.array(
                [
                    dx_pred - dx_meas,
                    dy_pred - dy_meas,
                    self._normalize_angle(dth_pred - dth_meas),
                ]
            )

            # Step 5: Weight by sqrt of information matrix
            # For diagonal info matrices: sqrt(diag(a,b,c)) = diag(√a,√b,√c)
            # This works because scipy minimizes Σ rᵢ², and we want to
            # minimize Σ eᵢᵀ Ω eᵢ = Σ (√Ω eᵢ)ᵀ(√Ω eᵢ) = Σ ||√Ω eᵢ||²
            sqrt_info = np.sqrt(np.abs(edge.info_matrix))
            weighted_error = sqrt_info @ error

            residuals.extend(weighted_error.tolist())

        return np.array(residuals)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        """Normalize angle to [-π, π].

        Critical for the residual function — without this, the optimizer
        can chase phantom rotations (e.g., thinking 1° and 359° are 358° apart).

        Args:
            angle_rad: Angle in radians.

        Returns:
            Equivalent angle in [-π, π].
        """
        return (angle_rad + math.pi) % (2 * math.pi) - math.pi

    # ── Loop Closure Detection ─────────────────────────────────────

    def find_loop_candidates(
        self,
        current_pose: tuple[float, float, float],
        min_node_gap: int = 10,
        max_distance_cm: float = 50.0,
    ) -> list[int]:
        """Find previous nodes that are spatially close to the current pose.

        A loop closure candidate is a node that:
        1. Is at least min_node_gap nodes in the past (not recent neighbors)
        2. Is within max_distance_cm of the current position

        Args:
            current_pose: Current robot pose (x, y, heading_deg).
            min_node_gap: Minimum node ID gap to avoid matching neighbors.
            max_distance_cm: Maximum spatial distance to consider.

        Returns:
            List of candidate node IDs, sorted by distance (closest first).
        """
        cx, cy = current_pose[0], current_pose[1]
        latest_id = self._next_id - 1  # Most recent node ID

        # Find nodes that are old enough (>= min_node_gap behind latest)
        # and spatially close
        candidates = []
        for nid, node in self.nodes.items():
            if latest_id - nid < min_node_gap:
                continue  # Too recent — skip
            dist = math.hypot(node.pose[0] - cx, node.pose[1] - cy)
            if dist <= max_distance_cm:
                candidates.append((dist, nid))

        # Sort by distance (closest first)
        candidates.sort()
        return [nid for _, nid in candidates]

    # ── Map Rebuild ────────────────────────────────────────────────

    def rebuild_map(self, grid: OccupancyGrid) -> None:
        """Clear the occupancy grid and replay all scans at optimized poses.

        After optimization adjusts node poses, the map must be rebuilt
        from scratch to reflect the corrected trajectory. This eliminates
        ghosting artifacts from previously incorrect poses.

        Args:
            grid: The OccupancyGrid to rebuild (will be cleared first).
        """
        # Reset grid to all-unknown (log-odds = 0)
        grid.grid[:] = 0.0

        # Replay each scan at its (now corrected) pose
        for nid in sorted(self.nodes.keys()):
            node = self.nodes[nid]
            grid.update(node.scan_points, node.pose, free_rays=node.free_rays)

    # ── Utilities ──────────────────────────────────────────────────

    def get_latest_node_id(self) -> int | None:
        """Return the ID of the most recently added node, or None if empty."""
        if not self.nodes:
            return None
        return self._next_id - 1

    def get_corrected_pose(self, node_id: int) -> tuple[float, float, float]:
        """Return the (possibly optimized) pose of a node.

        Args:
            node_id: Node ID to query.

        Returns:
            (x, y, heading_deg) — the current pose stored in the node.
        """
        return self.get_node(node_id).pose
