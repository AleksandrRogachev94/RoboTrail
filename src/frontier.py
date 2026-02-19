"""Frontier-based exploration for occupancy grid maps.

Finds boundaries between explored (free) and unexplored (unknown) space,
clusters them, and picks the best target for the robot to drive toward.
"""

import math

import numpy as np
from scipy.ndimage import label

from robot.config import GRID_SIZE


def find_frontiers(grid) -> np.ndarray:
    """Find frontier cells: free cells adjacent to unknown space.

    A frontier cell is a cell that:
    - Is definitely free (probability < 0.3)
    - Has at least one unknown neighbor (probability near 0.5, i.e. log-odds ≈ 0)

    Uses vectorized neighbor checks for speed.

    Args:
        grid: OccupancyGrid instance.

    Returns:
        (N, 2) array of (row, col) frontier cell indices.
        Empty array if no frontiers found.
    """
    prob_map = grid.get_probability_map()

    free = prob_map < 0.3
    unknown = (prob_map > 0.4) & (prob_map < 0.6)

    # Check if any of 4 neighbors is unknown (shift in each direction)
    has_unknown_neighbor = (
        np.roll(unknown, 1, axis=0)
        | np.roll(unknown, -1, axis=0)
        | np.roll(unknown, 1, axis=1)
        | np.roll(unknown, -1, axis=1)
    )

    # Zero out edges to avoid wrap-around artifacts from np.roll
    has_unknown_neighbor[0, :] = False
    has_unknown_neighbor[-1, :] = False
    has_unknown_neighbor[:, 0] = False
    has_unknown_neighbor[:, -1] = False

    frontier_mask = free & has_unknown_neighbor
    coords = np.argwhere(frontier_mask)  # (N, 2) array of [row, col]

    return coords


def cluster_frontiers(frontier_cells, grid, min_size=3):
    """Cluster frontier cells into contiguous groups.

    Uses scipy connected-component labeling on a binary mask of frontier cells.

    Args:
        frontier_cells: (N, 2) array of (row, col) from find_frontiers.
        grid: OccupancyGrid instance (for coordinate conversion).
        min_size: Minimum number of cells for a cluster to be kept.

    Returns:
        List of dicts, each with:
            centroid_xy: (world_x, world_y) center of cluster
            size: number of cells in cluster
        Sorted by size descending.
    """
    if len(frontier_cells) == 0:
        return []

    # Build binary mask of frontier cells
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[frontier_cells[:, 0], frontier_cells[:, 1]] = True

    # Connected components (4-connectivity)
    labeled, num_features = label(mask)

    clusters = []
    for i in range(1, num_features + 1):
        cells = np.argwhere(labeled == i)
        if len(cells) < min_size:
            continue

        # Centroid in grid coords → world coords
        mean_row = cells[:, 0].mean()
        mean_col = cells[:, 1].mean()
        cx, cy = grid.grid_to_world(int(mean_row), int(mean_col))

        clusters.append(
            {
                "centroid_xy": (cx, cy),
                "size": len(cells),
            }
        )

    # Sort by size descending (biggest frontier first)
    clusters.sort(key=lambda c: c["size"], reverse=True)

    return clusters


def pick_frontier(clusters, pose, grid, offset_cm=20.0):
    """Pick the best frontier cluster and compute a safe target point.

    Scores each cluster by:
        score = size - 0.3 * distance_to_robot

    Then offsets the target toward the robot (into known-free space) so
    the robot doesn't drive right to the frontier boundary.

    Args:
        clusters: List of cluster dicts from cluster_frontiers.
        pose: (x, y, heading_deg) current robot pose.
        grid: OccupancyGrid instance (for path feasibility checks).
        offset_cm: How far to pull the target back from the frontier
                   toward the robot. Skipped if frontier is already
                   within this distance.

    Returns:
        (x, y) target in world coordinates, or None if no viable frontier.
    """
    if not clusters:
        return None

    rx, ry = pose[0], pose[1]
    best_score = -float("inf")
    best_target = None

    for cluster in clusters:
        cx, cy = cluster["centroid_xy"]
        dist = math.hypot(cx - rx, cy - ry)
        score = cluster["size"] - 0.3 * dist
        if score > best_score:
            best_score = score
            best_target = (cx, cy)

    if best_target is None:
        return None

    # Offset target toward robot (into explored space)
    tx, ty = best_target
    dx = tx - rx
    dy = ty - ry
    dist = math.hypot(dx, dy)

    if dist > offset_cm:
        scale = (dist - offset_cm) / dist
        tx = rx + dx * scale
        ty = ry + dy * scale

    return (tx, ty)
