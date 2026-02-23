"""Path planning for occupancy grid navigation.

Implements A* search and line-of-sight simplification.
"""

import heapq
import math

import numpy as np

from occupancy_grid import OccupancyGrid

# =============================================================================
# A* Search
# =============================================================================


def a_star(
    traversability: np.ndarray, start_rc: tuple, goal_rc: tuple
) -> list[tuple] | None:
    """Find shortest path on a boolean traversability grid using A*.

    Args:
        traversability: 2D boolean array. True = cell is safe to traverse.
        start_rc: (row, col) starting cell.
        goal_rc: (row, col) goal cell.

    Returns:
        List of (row, col) from start to goal, or None if no path exists.
    """
    rows, cols = traversability.shape
    sr, sc = start_rc
    gr, gc = goal_rc

    # Validate start and goal
    if not traversability[sr, sc]:
        return None  # Start is blocked
    if not traversability[gr, gc]:
        return None  # Goal is blocked

    # 8-connected neighbors: (delta_row, delta_col, step_cost)
    NEIGHBORS = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),  # Cardinal
        (-1, -1, 1.414),
        (-1, 1, 1.414),
        (1, -1, 1.414),
        (1, 1, 1.414),  # Diagonal
    ]

    def heuristic(r, c):
        """Euclidean distance to goal in grid cells."""
        return math.hypot(r - gr, c - gc)

    # 1. Initialize data structures
    g_score = {start_rc: 0}
    # Priority queue stores (f_score, cell); f_score = g_score + heuristic
    open_set = [(heuristic(*start_rc), start_rc)]
    came_from = {}

    while open_set:
        # Pop lowest f_score
        _, current = heapq.heappop(open_set)

        if current == goal_rc:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_rc)
            return list(reversed(path))

        r, c = current
        for dr, dc, step_cost in NEIGHBORS:
            nr, nc = r + dr, c + dc

            if not (0 <= nr < rows and 0 <= nc < cols):
                continue

            if not traversability[nr, nc]:
                continue

            tentative_g = g_score[current] + step_cost

            # If path to neighbor is better than any previous one
            if tentative_g < g_score.get((nr, nc), float("inf")):
                came_from[(nr, nc)] = current
                g_score[(nr, nc)] = tentative_g
                f_score = tentative_g + heuristic(nr, nc)
                heapq.heappush(open_set, (f_score, (nr, nc)))

    return None


# =============================================================================
# Line-of-Sight Simplification
# =============================================================================


def _line_of_sight(
    traversability: np.ndarray, r0: int, c0: int, r1: int, c1: int
) -> bool:
    """Check if all cells along a Bresenham line are traversable.

    Args:
        traversability: 2D boolean grid.
        r0, c0: Start cell.
        r1, c1: End cell.

    Returns:
        True if every cell on the line is traversable.
    """
    ray = OccupancyGrid.bresenham(r0, c0, r1, c1)
    for r, c in ray:
        if not traversability[r, c]:
            return False
    return True


def simplify_path(path: list[tuple], traversability: np.ndarray) -> list[tuple]:
    """Remove redundant waypoints using line-of-sight checks.

    Args:
        path: List of (row, col) from A*.
        traversability: 2D boolean grid.

    Returns:
        Reduced list of (row, col) — only the essential corner waypoints.
    """
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    current = 0
    while current < len(path) - 1:
        # Try to skip as far ahead as possible
        farthest = current + 1
        for candidate in range(current + 2, len(path)):
            if _line_of_sight(traversability, *path[current], *path[candidate]):
                farthest = candidate
        simplified.append(path[farthest])
        current = farthest
    return simplified


# =============================================================================
# Top-Level API
# =============================================================================


def plan_path(
    grid,  # OccupancyGrid instance
    start_xy: tuple,
    goal_xy: tuple,
    obstacle_inflation: int = 7,
    free_inflation: int = 5,
) -> list[tuple] | None:
    """Path planning pipeline: A* → simplify → world coordinates.

    Args:
        grid: OccupancyGrid instance.
        start_xy: (x, y) start position in world cm.
        goal_xy: (x, y) goal position in world cm.
        obstacle_inflation: Cells to inflate obstacles by.
        free_inflation: Cells to inflate free space by.

    Returns:
        List of (x, y) world-coordinate waypoints, or None if no path.
    """
    # Step 1: Get traversability grid
    traversable = grid.get_traversability_grid(obstacle_inflation, free_inflation)

    # Step 2: Convert start/goal to grid coordinates
    start_rc = grid.world_to_grid(*start_xy)
    goal_rc = grid.world_to_grid(*goal_xy)

    # Step 2b: Ensure start is traversable (robot is physically here,
    # but ICP correction may have placed it inside an inflation zone)
    sr, sc = start_rc
    rows, cols = traversable.shape
    CLEAR_RADIUS = 6  # cells (~12cm) — enough to bridge inflation gap
    r_lo = max(0, sr - CLEAR_RADIUS)
    r_hi = min(rows, sr + CLEAR_RADIUS + 1)
    c_lo = max(0, sc - CLEAR_RADIUS)
    c_hi = min(cols, sc + CLEAR_RADIUS + 1)
    traversable[r_lo:r_hi, c_lo:c_hi] = True

    # Step 3: A* search
    raw_path = a_star(traversable, start_rc, goal_rc)
    if raw_path is None:
        return None

    # Step 4: Simplify (remove redundant waypoints via line-of-sight)
    simplified = simplify_path(raw_path, traversable)

    # Step 5: Convert to world coordinates
    waypoints = [grid.grid_to_world(r, c) for r, c in simplified]

    return waypoints
