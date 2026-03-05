"""Frontier detection and goal selection for autonomous exploration.

Frontiers are boundaries between traversable and unknown space.
The robot explores by driving toward frontier goals, mapping as it goes.
"""

import math
from collections import deque

import numpy as np

from occupancy_grid import OccupancyGrid
from path_planner import a_star
from robot.config import GRID_RESOLUTION

# ── Constants ──────────────────────────────────────────────────────────

MIN_CLUSTER_SIZE = 80  # Filter small gaps; real frontiers are larger

# 8-connected neighbors for adjacency checks and BFS
_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


# ── Frontier Detection ────────────────────────────────────────────────


def find_frontiers(grid: OccupancyGrid) -> list[list[tuple[int, int]]]:
    """Detect frontier cells and cluster them via BFS flood-fill.

    A frontier cell is a traversable cell adjacent to at least one
    unknown cell (log-odds ≈ 0, never observed).

    Returns:
        Clusters sorted by size (largest first), each a list of (row, col).
        Clusters smaller than MIN_CLUSTER_SIZE are filtered out.
    """
    log_odds = grid.grid
    rows, cols = log_odds.shape

    # Traversability grid bridges gaps between scan rays (free inflation)
    # and stays away from walls (obstacle inflation).
    traversable = grid.get_traversability_grid()
    unknown_mask = np.abs(log_odds) < 0.1

    # Frontier = traversable cell with at least one unknown neighbor
    frontier_mask = np.zeros((rows, cols), dtype=bool)
    for dr, dc in _NEIGHBORS:
        shifted = np.zeros_like(unknown_mask)
        sr = slice(max(0, -dr), rows + min(0, -dr))
        sc = slice(max(0, -dc), cols + min(0, -dc))
        dr_dest = slice(max(0, dr), rows + min(0, dr))
        dc_dest = slice(max(0, dc), cols + min(0, dc))
        shifted[dr_dest, dc_dest] = unknown_mask[sr, sc]
        frontier_mask |= shifted
    frontier_mask &= traversable

    # BFS flood-fill to cluster connected frontier cells
    frontier_cells = set(zip(*np.where(frontier_mask)))
    visited = set()
    clusters = []

    for cell in frontier_cells:
        if cell in visited:
            continue
        cluster = []
        queue = deque([cell])
        visited.add(cell)
        while queue:
            r, c = queue.popleft()
            cluster.append((r, c))
            for dr, dc in _NEIGHBORS:
                nb = (r + dr, c + dc)
                if nb in frontier_cells and nb not in visited:
                    visited.add(nb)
                    queue.append(nb)
        if len(cluster) >= MIN_CLUSTER_SIZE:
            clusters.append(cluster)

    clusters.sort(key=len, reverse=True)
    return clusters


# ── Goal Selection ─────────────────────────────────────────────────────


def select_goal(
    grid: OccupancyGrid,
    clusters: list[list[tuple[int, int]]],
    robot_pose: tuple[float, float, float],
    min_distance_cm: float = 0,
) -> tuple[float, float] | None:
    """Pick the best frontier goal from detected clusters.

    Scores each cluster by path distance + heading penalty (0.5cm per
    degree of turn required). Prefers goals roughly ahead of the robot
    to minimize large rotations that break ICP overlap.
    The goal is the closest point in the cluster to the robot.

    Args:
        min_distance_cm: Skip goals closer than this (avoids picking
                         frontiers the robot is already sitting on).

    Returns:
        (goal_x, goal_y) in world cm, or None if nothing reachable.
    """
    if not clusters:
        return None

    rx, ry, heading_deg = robot_pose
    traversable = grid.get_traversability_grid()
    start_rc = grid.world_to_grid(rx, ry)

    # Ensure start is traversable (robot is physically here)
    sr, sc = start_rc
    rows, cols = traversable.shape
    CLEAR_R = 6
    r_lo, r_hi = max(0, sr - CLEAR_R), min(rows, sr + CLEAR_R + 1)
    c_lo, c_hi = max(0, sc - CLEAR_R), min(cols, sc + CLEAR_R + 1)
    traversable[r_lo:r_hi, c_lo:c_hi] = True

    best_goal = None
    best_score = float("inf")

    for cluster in clusters:
        # Find the point in the cluster closest to the robot
        gx, gy = None, None
        min_cluster_dist = float("inf")
        for r, c in cluster:
            wx, wy = grid.grid_to_world(r, c)
            d = math.hypot(wx - rx, wy - ry)
            if d < min_cluster_dist:
                min_cluster_dist = d
                gx, gy = wx, wy

        if gx is None or min_cluster_dist < max(1.0, min_distance_cm):
            continue

        # Verify goal is on traversable space
        goal_rc = grid.world_to_grid(gx, gy)
        gr, gc = goal_rc
        if not (0 <= gr < rows and 0 <= gc < cols and traversable[gr, gc]):
            continue

        # Score = path distance + heading penalty
        path = a_star(traversable, start_rc, goal_rc)
        if path is None:
            continue

        # Penalize goals that require large heading changes —
        # reduces unnecessary rotations that break ICP overlap
        goal_heading = math.degrees(math.atan2(gy - ry, gx - rx))
        heading_diff = abs((goal_heading - heading_deg + 180) % 360 - 180)
        heading_penalty = heading_diff * 0.5  # 0.5cm-equivalent per degree
        score = len(path) * GRID_RESOLUTION + heading_penalty

        if score < best_score:
            best_score = score
            best_goal = (gx, gy)

    return best_goal


# ── Visualization Data ─────────────────────────────────────────────────


def get_frontier_viz_data(
    grid: OccupancyGrid,
    clusters: list[list[tuple[int, int]]],
    robot_pose: tuple[float, float, float],
) -> list[dict]:
    """Return frontier data formatted for the web UI.

    Each entry has edge_pt (closest cell to robot), size, and cell positions.
    """
    rx, ry, _ = robot_pose
    result = []
    for cluster in clusters:
        # Find closest point to robot
        cx, cy = None, None
        min_dist = float("inf")
        cells = []
        for r, c in cluster:
            wx, wy = grid.grid_to_world(r, c)
            cells.append([float(round(wx, 1)), float(round(wy, 1))])

            d = math.hypot(wx - rx, wy - ry)
            if d < min_dist:
                min_dist = d
                cx, cy = wx, wy

        result.append(
            {
                "edge_pt": [float(round(cx, 1)), float(round(cy, 1))],
                "size": len(cluster),
                "cells": cells,
            }
        )

    return result
