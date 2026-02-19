"""Frontier detection and goal selection for autonomous exploration.

Frontiers are boundaries between traversable and unknown space.
The robot explores by driving toward frontier goals, mapping as it goes.

Design:
  - Detection uses the traversability grid (inflated free space) to avoid
    fragmented frontiers between sparse scan rays.
  - Standoff: target a point pulled back from the frontier centroid so the
    robot has room to arc-maneuver at the destination.
  - Forward bias: penalize frontiers behind the robot to avoid costly U-turns.
  - Commitment: pick one goal per iteration; re-evaluate only on arrival.
"""

import math
from collections import deque

import numpy as np

from occupancy_grid import OccupancyGrid
from path_planner import a_star
from robot.config import GRID_RESOLUTION

# ── Constants ──────────────────────────────────────────────────────────

MIN_CLUSTER_SIZE = 30  # Filter small interior holes; real frontiers are 100+ cells
STANDOFF_CM = 15.0  # Pull goal back from frontier edge (arc room)
MIN_FRONTIER_DIST = 15.0 
HEADING_PENALTY_WEIGHT = 0.8  # cm per degree — strongly prefer forward-facing goals

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
) -> tuple[float, float] | None:
    """Pick the best frontier goal from detected clusters.

    Scores each cluster by: path_distance + heading_penalty.
    The goal point is pulled back from the centroid by STANDOFF_CM.

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
        cx, cy = None, None
        min_cluster_dist = float("inf")
        for r, c in cluster:
            wx, wy = grid.grid_to_world(r, c)
            d = math.hypot(wx - rx, wy - ry)
            if d < min_cluster_dist:
                min_cluster_dist = d
                cx, cy = wx, wy

        # Pull goal toward robot by STANDOFF_CM
        dx, dy = cx - rx, cy - ry
        dist = math.hypot(dx, dy)
        if dist < 1.0:
            continue

        if dist > STANDOFF_CM:
            ratio = (dist - STANDOFF_CM) / dist
            gx, gy = rx + dx * ratio, ry + dy * ratio
        else:
            gx, gy = cx, cy

        # Verify goal is on traversable space
        goal_rc = grid.world_to_grid(gx, gy)
        gr, gc = goal_rc
        if not (0 <= gr < rows and 0 <= gc < cols and traversable[gr, gc]):
            # Fall back to the edge point if standoff point is blocked
            goal_rc = grid.world_to_grid(cx, cy)
            gr, gc = goal_rc
            if not (0 <= gr < rows and 0 <= gc < cols and traversable[gr, gc]):
                continue
            gx, gy = cx, cy

        # Score = path distance + heading penalty
        path = a_star(traversable, start_rc, goal_rc)
        if path is None:
            continue

        path_dist = len(path) * GRID_RESOLUTION
        angle_to_goal = math.degrees(math.atan2(dy, dx))
        heading_diff = abs((angle_to_goal - heading_deg + 180) % 360 - 180)
        score = path_dist + heading_diff * HEADING_PENALTY_WEIGHT

        if score < best_score:
            best_score = score
            best_goal = (gx, gy)

    if best_goal and math.hypot(best_goal[0] - rx, best_goal[1] - ry) < MIN_FRONTIER_DIST:
        return None  # force re-evaluation rather than targeting something we're on top of

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
