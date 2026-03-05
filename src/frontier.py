"""Frontier detection and goal selection for autonomous exploration.

Frontiers are boundaries between explored (free) and unknown space.
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

# Small obstacle inflation for frontier detection (~4cm at 2cm resolution).
# Just enough to prevent free_inflation from leaking through thin walls,
# but much less than the full robot-clearance inflation used for path planning.
_DETECTION_OBSTACLE_INFLATION = 2

# 8-connected neighbors
_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


# ── Frontier Detection ────────────────────────────────────────────────


def find_frontiers(grid: OccupancyGrid) -> list[list[tuple[int, int]]]:
    """Detect frontier cells and cluster them via BFS flood-fill.

    A frontier cell is a free-inflated cell adjacent to at least one
    unknown cell. Uses small obstacle inflation to prevent leaking
    through wall gaps, but much less than path-planning inflation.

    Returns:
        Clusters sorted by size (largest first), each a list of (row, col).
        Clusters smaller than MIN_CLUSTER_SIZE are filtered out.
    """
    log_odds = grid.grid
    rows, cols = log_odds.shape

    traversable = grid.get_traversability_grid(
        obstacle_inflation=_DETECTION_OBSTACLE_INFLATION
    )
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


def _snap_to_traversable(
    traversable: np.ndarray,
    row: int,
    col: int,
    max_search: int = 200,
) -> tuple[int, int] | None:
    """Find the nearest traversable cell to (row, col) via BFS."""
    rows, cols = traversable.shape
    if 0 <= row < rows and 0 <= col < cols and traversable[row, col]:
        return (row, col)

    queue = deque([(row, col)])
    visited = {(row, col)}
    while queue and len(visited) < max_search:
        r, c = queue.popleft()
        for dr, dc in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if (nr, nc) in visited:
                continue
            visited.add((nr, nc))
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if traversable[nr, nc]:
                return (nr, nc)
            queue.append((nr, nc))
    return None


def select_goal(
    grid: OccupancyGrid,
    clusters: list[list[tuple[int, int]]],
    robot_pose: tuple[float, float, float],
    min_distance_cm: float = 0,
) -> tuple[float, float] | None:
    """Pick the best frontier goal from detected clusters.

    For each cluster, finds the closest point to the robot that is at
    least min_distance_cm away (approaches frontier from explored side).
    Snaps to nearest traversable cell for safe path planning.

    Args:
        min_distance_cm: Skip points closer than this (avoids picking
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
        # Find the closest cluster point that's at least min_distance_cm away.
        # (Don't skip the whole cluster if one edge is too close.)
        gx, gy = None, None
        best_dist = float("inf")
        min_dist_threshold = max(1.0, min_distance_cm)
        for r, c in cluster:
            wx, wy = grid.grid_to_world(r, c)
            d = math.hypot(wx - rx, wy - ry)
            if d >= min_dist_threshold and d < best_dist:
                best_dist = d
                gx, gy = wx, wy

        if gx is None:
            continue

        # Snap to nearest traversable cell for path planning
        goal_rc = grid.world_to_grid(gx, gy)
        snap = _snap_to_traversable(traversable, goal_rc[0], goal_rc[1])
        if snap is None:
            continue
        goal_rc = snap
        gx, gy = grid.grid_to_world(snap[0], snap[1])

        # Path plan with full-inflation traversability
        path = a_star(traversable, start_rc, goal_rc)
        if path is None:
            continue

        # Score = path distance + heading penalty
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

    Each entry has edge_pt (closest point to robot), size, and cell positions.
    """
    rx, ry, _ = robot_pose
    result = []
    for cluster in clusters:
        cells = []
        cx, cy = None, None
        min_dist = float("inf")
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
