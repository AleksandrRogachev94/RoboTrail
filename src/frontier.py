"""Frontier detection and goal selection for autonomous exploration.

Frontiers are boundaries between known-free and unknown space.
The robot explores by driving toward frontier goals, mapping as it goes.

Key design decisions:
  - Standoff: target 15cm before frontier centroid (room to maneuver arcs)
  - Commitment: pick one goal and drive to it; re-evaluate only on arrival/block
  - Forward bias: penalize frontiers behind the robot to avoid costly U-turns
"""

import math
from collections import deque

import numpy as np

from occupancy_grid import OccupancyGrid
from path_planner import a_star

# ── Constants ──────────────────────────────────────────────────────────

MIN_CLUSTER_SIZE = 5  # Ignore frontier clusters smaller than this (noise)
STANDOFF_CM = 15.0  # Don't drive all the way to frontier — leave room for arcs
HEADING_PENALTY_WEIGHT = 0.3  # cm per degree of heading deviation
MIN_DRIVE_CM = 15.0  # If frontier is closer than this, drive forward instead

# 8-connected neighbors
_NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]


# ── Frontier Detection ────────────────────────────────────────────────


def find_frontiers(grid: OccupancyGrid) -> list[list[tuple[int, int]]]:
    """Detect frontier cells and cluster them via BFS flood-fill.

    A frontier cell is a FREE cell (P < 0.3) adjacent to at least one
    UNKNOWN cell (log-odds ≈ 0, never observed).

    Args:
        grid: The occupancy grid with current map data.

    Returns:
        List of clusters. Each cluster is a list of (row, col) grid cells.
        Clusters smaller than MIN_CLUSTER_SIZE are filtered out.
        Sorted by cluster size (largest first).
    """
    prob_map = grid.get_probability_map()
    log_odds = grid.grid
    rows, cols = log_odds.shape

    # Build masks
    free_mask = prob_map < 0.3
    unknown_mask = np.abs(log_odds) < 0.1  # Never observed

    # Find frontier cells: free cells with at least one unknown neighbor
    # Use convolution-like approach for speed
    frontier_mask = np.zeros_like(free_mask)

    for dr, dc in _NEIGHBORS:
        # Shift unknown mask and check overlap with free
        shifted = np.zeros_like(unknown_mask)
        # Source slice (where we read from unknown_mask)
        sr = slice(max(0, -dr), rows + min(0, -dr))
        sc = slice(max(0, -dc), cols + min(0, -dc))
        # Destination slice (where we write to shifted)
        dr_dest = slice(max(0, dr), rows + min(0, dr))
        dc_dest = slice(max(0, dc), cols + min(0, dc))
        shifted[dr_dest, dc_dest] = unknown_mask[sr, sc]
        frontier_mask |= shifted

    frontier_mask &= free_mask

    # BFS flood-fill to cluster connected frontier cells
    frontier_cells = set(zip(*np.where(frontier_mask)))
    visited = set()
    clusters = []

    for cell in frontier_cells:
        if cell in visited:
            continue

        # BFS from this cell
        cluster = []
        queue = deque([cell])
        visited.add(cell)

        while queue:
            r, c = queue.popleft()
            cluster.append((r, c))

            for dr, dc in _NEIGHBORS:
                nr, nc = r + dr, c + dc
                neighbor = (nr, nc)
                if neighbor in frontier_cells and neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)

        if len(cluster) >= MIN_CLUSTER_SIZE:
            clusters.append(cluster)

    # Sort largest first
    clusters.sort(key=len, reverse=True)
    return clusters


# ── Goal Selection ─────────────────────────────────────────────────────


def select_goal(
    grid: OccupancyGrid,
    clusters: list[list[tuple[int, int]]],
    robot_pose: tuple[float, float, float],
) -> tuple[float, float] | None:
    """Select the best frontier goal from detected clusters.

    For each cluster:
    1. Compute centroid in world coordinates
    2. Pull goal 15cm back toward robot (standoff)
    3. Score = path_distance + heading_penalty
    4. Pick lowest score among reachable goals

    Args:
        grid: Occupancy grid for path planning.
        clusters: Frontier clusters from find_frontiers().
        robot_pose: (x, y, heading_deg) current robot pose.

    Returns:
        (goal_x, goal_y) in world cm, or None if no reachable frontier.
    """
    if not clusters:
        return None

    rx, ry, heading_deg = robot_pose

    traversable = grid.get_traversability_grid()
    start_rc = grid.world_to_grid(rx, ry)

    # Ensure start is traversable (robot is physically here)
    sr, sc = start_rc
    rows, cols = traversable.shape
    CLEAR_RADIUS = 5
    r_lo = max(0, sr - CLEAR_RADIUS)
    r_hi = min(rows, sr + CLEAR_RADIUS + 1)
    c_lo = max(0, sc - CLEAR_RADIUS)
    c_hi = min(cols, sc + CLEAR_RADIUS + 1)
    traversable[r_lo:r_hi, c_lo:c_hi] = True

    best_goal = None
    best_score = float("inf")

    for cluster in clusters:
        # Compute centroid in grid coords
        cr = sum(r for r, c in cluster) / len(cluster)
        cc = sum(c for r, c in cluster) / len(cluster)

        # Convert to world
        cx, cy = grid.grid_to_world(cr, cc)

        # Apply standoff: pull goal toward robot
        dx = cx - rx
        dy = cy - ry
        dist = math.hypot(dx, dy)

        if dist < 1.0:
            continue  # Skip clusters on top of robot

        if dist > STANDOFF_CM:
            # Pull back by STANDOFF_CM
            ratio = (dist - STANDOFF_CM) / dist
            gx = rx + dx * ratio
            gy = ry + dy * ratio
        else:
            # Too close — goal is at robot position, skip
            # (handled by MIN_DRIVE_CM logic in explorer)
            gx, gy = cx, cy

        # Check if goal is on traversable space
        goal_rc = grid.world_to_grid(gx, gy)
        gr, gc = goal_rc
        if not (0 <= gr < rows and 0 <= gc < cols and traversable[gr, gc]):
            # Try the centroid directly
            goal_rc = (int(round(cr)), int(round(cc)))
            gr, gc = goal_rc
            if not (0 <= gr < rows and 0 <= gc < cols and traversable[gr, gc]):
                continue
            gx, gy = cx, cy

        # Score: path distance + heading penalty
        path = a_star(traversable, start_rc, goal_rc)
        if path is None:
            continue

        # Path distance in cm (each cell = GRID_RESOLUTION cm)
        from robot.config import GRID_RESOLUTION

        path_dist = len(path) * GRID_RESOLUTION

        # Heading penalty: angle between current heading and direction to goal
        angle_to_goal = math.degrees(math.atan2(dy, dx))
        heading_diff = abs((angle_to_goal - heading_deg + 180) % 360 - 180)
        penalty = heading_diff * HEADING_PENALTY_WEIGHT

        score = path_dist + penalty

        if score < best_score:
            best_score = score
            best_goal = (gx, gy)

    return best_goal


# ── Visualization Data ─────────────────────────────────────────────────


def get_frontier_viz_data(
    grid: OccupancyGrid, clusters: list[list[tuple[int, int]]]
) -> list[dict]:
    """Return frontier data formatted for the web UI.

    Args:
        grid: Occupancy grid (for coordinate conversion).
        clusters: Frontier clusters from find_frontiers().

    Returns:
        List of dicts with centroid (x, y), size, and boundary cells
        for overlay rendering.
    """
    result = []
    for cluster in clusters:
        cr = sum(r for r, c in cluster) / len(cluster)
        cc = sum(c for r, c in cluster) / len(cluster)
        cx, cy = grid.grid_to_world(cr, cc)

        # Also include all frontier cell positions for rendering
        cells = []
        for r, c in cluster:
            wx, wy = grid.grid_to_world(r, c)
            cells.append([float(round(wx, 1)), float(round(wy, 1))])

        result.append(
            {
                "centroid": [float(round(cx, 1)), float(round(cy, 1))],
                "size": len(cluster),
                "cells": cells,
            }
        )

    return result
