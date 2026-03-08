"""Tests for pose graph SLAM.

Test outline — implement these as you build pose_graph.py.
Run: python3 -m pytest pose_graph_test.py -v
"""

import math

import numpy as np
import pytest

from pose_graph import PoseGraph

# ── Basic Graph Operations ────────────────────────────────────────────


class TestGraphBasics:
    """Test node/edge management."""

    def test_add_node_returns_sequential_ids(self):
        """add_node should return 0, 1, 2, ..."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0], [0.0, 10.0]])
        n0 = graph.add_node((0, 0, 0), scan)
        n1 = graph.add_node((10, 0, 0), scan)
        assert n0 == 0
        assert n1 == 1
        assert graph.num_nodes == 2

    def test_get_node(self):
        """get_node should return the stored PoseNode."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])
        graph.add_node((5.0, 3.0, 45.0), scan)
        node = graph.get_node(0)
        assert node.pose == (5.0, 3.0, 45.0)
        assert np.array_equal(node.scan_points, scan)

    def test_add_edge(self):
        """add_edge should store the edge."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])
        graph.add_node((0, 0, 0), scan)
        graph.add_node((10, 0, 0), scan)

        info = np.eye(3)
        graph.add_edge(0, 1, (10, 0, 0), "odometry", info)
        assert graph.num_edges == 1

    def test_add_edge_invalid_node_raises(self):
        """add_edge with nonexistent node should raise KeyError."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])
        graph.add_node((0, 0, 0), scan)
        with pytest.raises(KeyError):
            graph.add_edge(0, 99, (10, 0, 0), "odometry", np.eye(3))


# ── Information Matrices ──────────────────────────────────────────────


class TestInformationMatrices:
    """Test confidence weighting."""

    def test_odometry_info_higher_for_short_moves(self):
        """Shorter moves should produce higher confidence (larger info values)."""
        info_short = PoseGraph.compute_odometry_info_matrix(5.0, 10.0)
        info_long = PoseGraph.compute_odometry_info_matrix(50.0, 90.0)
        # Diagonal values should be larger for short moves
        assert info_short[0, 0] > info_long[0, 0]

    def test_icp_info_higher_for_good_match(self):
        """Good ICP match should produce higher confidence."""
        info_good = PoseGraph.compute_icp_info_matrix(0.9, 1.0, True)
        info_bad = PoseGraph.compute_icp_info_matrix(0.3, 8.0, True)
        assert info_good[0, 0] > info_bad[0, 0]

    def test_icp_info_zero_for_failed(self):
        """Failed ICP should produce zero or near-zero info matrix."""
        info = PoseGraph.compute_icp_info_matrix(0.2, 10.0, False)
        assert np.allclose(info, 0) or info[0, 0] < 0.01


# ── Angle Normalization ────────────────────────────────────────────────


class TestNormalizeAngle:
    """Test angle wrapping — critical for optimizer correctness."""

    def test_already_normalized(self):
        assert abs(PoseGraph._normalize_angle(0.5) - 0.5) < 1e-10

    def test_wrap_positive(self):
        """2π + 0.1 should wrap to ~0.1"""
        result = PoseGraph._normalize_angle(2 * math.pi + 0.1)
        assert abs(result - 0.1) < 1e-10

    def test_wrap_negative(self):
        """-2π - 0.1 should wrap to ~-0.1"""
        result = PoseGraph._normalize_angle(-2 * math.pi - 0.1)
        assert abs(result - (-0.1)) < 1e-10

    def test_pi_boundary(self):
        """π should stay as π (or -π, both are valid)."""
        result = PoseGraph._normalize_angle(math.pi)
        assert abs(abs(result) - math.pi) < 1e-10


# ── Optimization ──────────────────────────────────────────────────────


class TestOptimization:
    """Test the scipy-based optimizer."""

    def test_perfect_chain_unchanged(self):
        """A chain with perfect edges should not change poses."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0], [0.0, 10.0], [10.0, 10.0]])

        graph.add_node((0, 0, 0), scan)
        graph.add_node((10, 0, 0), scan)
        graph.add_node((20, 0, 0), scan)

        info = np.eye(3) * 100  # High confidence
        graph.add_edge(0, 1, (10, 0, 0), "odometry", info)
        graph.add_edge(1, 2, (10, 0, 0), "odometry", info)

        graph.optimize()

        # Poses should be (nearly) unchanged
        p1 = graph.get_corrected_pose(1)
        assert abs(p1[0] - 10.0) < 0.5
        assert abs(p1[1] - 0.0) < 0.5

    def test_conflicting_edges_compromise(self):
        """With conflicting edges, optimizer should find a compromise."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])

        graph.add_node((0, 0, 0), scan)
        graph.add_node((10, 0, 0), scan)

        # Odometry says 10cm, ICP says 12cm
        info = np.eye(3) * 100
        graph.add_edge(0, 1, (10, 0, 0), "odometry", info)
        graph.add_edge(0, 1, (12, 0, 0), "icp_sequential", info)

        graph.optimize()

        # Should compromise ~11cm
        p1 = graph.get_corrected_pose(1)
        assert 10.0 < p1[0] < 12.0

    def test_bad_edge_downweighted_by_huber(self):
        """A wildly wrong edge should get Huber-downweighted."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])

        graph.add_node((0, 0, 0), scan)
        graph.add_node((10, 0, 0), scan)
        graph.add_node((20, 0, 0), scan)

        info = np.eye(3) * 100
        graph.add_edge(0, 1, (10, 0, 0), "odometry", info)
        graph.add_edge(1, 2, (10, 0, 0), "odometry", info)

        # Bad ICP edge says node 2 is at x=50 (wildly wrong)
        graph.add_edge(0, 2, (50, 0, 0), "icp_sequential", info * 0.1)

        graph.optimize()

        # Node 2 should stay near x=20, not jump to x=50
        p2 = graph.get_corrected_pose(2)
        assert abs(p2[0] - 20.0) < 5.0

    def test_square_loop_closure(self):
        """Robot drives a square. Loop closure should correct drift.

         N0 ──(10,0)──► N1
         ▲                │
         │                (0,10)
        (0,-10)           │
         │                ▼
         N3 ◄──(10,0)── N2

         Add nodes with slight drift, then a loop closure edge
         constraining N3 back to N0. Verify poses form a clean square.
        """
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0], [0.0, 10.0]])

        # True square: (0,0) → (10,0) → (10,10) → (0,10) → (0,0)
        # But add drift: each step accumulates ~1cm error
        graph.add_node((0, 0, 0), scan)  # N0
        graph.add_node((10, 0.5, 0), scan)  # N1 (drifted 0.5cm in y)
        graph.add_node((10.5, 10, 90), scan)  # N2
        graph.add_node((-0.5, 10.5, 180), scan)  # N3

        info = np.eye(3) * 100
        graph.add_edge(0, 1, (10, 0, 0), "odometry", info)
        graph.add_edge(1, 2, (0, 10, 90), "odometry", info)
        graph.add_edge(2, 3, (-10, 0, 90), "odometry", info)

        # Loop closure: N3 back to N0
        graph.add_edge(3, 0, (0, -10, 90), "loop_closure", info * 2)

        graph.optimize()

        # After optimization, poses should be closer to a perfect square
        p1 = graph.get_corrected_pose(1)
        assert abs(p1[1]) < 0.5  # y drift should be corrected


# ── Loop Closure Detection ────────────────────────────────────────────


class TestLoopClosure:
    """Test loop closure candidate finding."""

    def test_finds_nearby_old_node(self):
        """Should find nodes that are spatially close but temporally distant."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])

        # Create 15 nodes in a line
        for i in range(15):
            graph.add_node((i * 10, 0, 0), scan)

        # Current pose is near node 0
        candidates = graph.find_loop_candidates(
            current_pose=(5, 0, 0),
            min_node_gap=10,
            max_distance_cm=50,
        )
        assert 0 in candidates

    def test_ignores_recent_nodes(self):
        """Should not return nodes within min_node_gap."""
        graph = PoseGraph()
        scan = np.array([[10.0, 0.0]])

        for i in range(5):
            graph.add_node((0, 0, 0), scan)  # All at same position

        # Node 4 is the latest; min_gap=10 means no candidates
        candidates = graph.find_loop_candidates(
            current_pose=(0, 0, 0),
            min_node_gap=10,
            max_distance_cm=50,
        )
        assert len(candidates) == 0
