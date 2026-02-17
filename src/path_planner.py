"""Path planning and smoothing for occupancy grid navigation.

Implements A* search, line-of-sight simplification, gradient descent smoothing,
and adaptive waypoint resampling.
"""

import heapq
import math

import numpy as np

from occupancy_grid import OccupancyGrid

# =============================================================================
# Sub-problem 1: A* Search
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
# Sub-problem 2: Line-of-Sight Simplification
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
# Sub-problem 3: Gradient Descent Smoothing
# =============================================================================


def smooth_path(
    path: list[tuple],
    weight_data: float = 0.2,
    weight_smooth: float = 0.2,
    tolerance: float = 0.001,
    max_iterations: int = 1000,
) -> list[tuple]:
    """Smooth a path using gradient descent (Sebastian Thrun's method).

    Balances two competing forces on each interior point:
    - DATA force: pulls point back toward original position (keeps path faithful)
    - SMOOTH force: pulls point toward average of its neighbors (reduces zig-zag)

    The first and last points are ANCHORED (never move).

    Args:
        path: List of (row, col) waypoints — already simplified.
        weight_data: Strength of "stay close to original" force.
        weight_smooth: Strength of "be smooth" force.
        tolerance: Stop when max change per iteration drops below this.
        max_iterations: Safety limit to prevent infinite loops.

    Returns:
        Smoothed list of (row, col) as floats.
    """
    if len(path) <= 2:
        return list(path)  # Nothing to smooth

    # 1. Create a mutable copy: smooth = [[r, c] for r, c in path]
    #    Also keep original = [[r, c] for r, c in path]
    smooth = [[r, c] for r, c in path]

    # 2. Iterate up to max_iterations:
    j = 0
    while j < max_iterations:
        j += 1
        max_change = 0
        for i in range(1, len(path) - 1):
            for d in range(2):
                data_pull = weight_data * (path[i][d] - smooth[i][d])
                smooth_pull = weight_smooth * (
                    smooth[i - 1][d] + smooth[i + 1][d] - 2 * smooth[i][d]
                )
                change = data_pull + smooth_pull
                smooth[i][d] += change
                max_change = max(max_change, abs(change))
        if max_change < tolerance:
            break

    return smooth


# =============================================================================
# Sub-problem 4: Adaptive Waypoint Resampling
# =============================================================================


def sample_waypoints(
    smooth_path_pts: list[tuple],
    grid_to_world_fn,
    min_spacing_cm: float = 10.0,
    max_spacing_cm: float = 30.0,
    heading_threshold_deg: float = 15.0,
) -> list[tuple]:
    """Resample smoothed path into world-coordinate waypoints with adaptive density.

    Args:
        smooth_path_pts: Smoothed path in grid coordinates [(row, col), ...].
        grid_to_world_fn: Function(row, col) → (world_x, world_y) in cm.
        min_spacing_cm: Minimum distance between waypoints (prevent thrashing).
        max_spacing_cm: Maximum distance between waypoints (ensure regular scans).
        heading_threshold_deg: Place waypoint when heading change exceeds this.

    Returns:
        List of (world_x, world_y) waypoints in cm.
    """
    if len(smooth_path_pts) < 2:
        r, c = smooth_path_pts[0]
        return [grid_to_world_fn(r, c)]

    # 1. Convert to world coordinates:
    world_pts = [grid_to_world_fn(r, c) for r, c in smooth_path_pts]

    # Always include the start
    waypoints = [world_pts[0]]

    dist_accum = 0.0
    heading_accum = 0.0

    # Initial heading from first segment
    p0 = world_pts[0]
    p1 = world_pts[1]
    prev_heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

    for i in range(1, len(world_pts)):
        p_prev = world_pts[i - 1]
        p_curr = world_pts[i]

        # Distance
        step_dist = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
        dist_accum += step_dist

        # Heading change
        curr_heading = math.atan2(p_curr[1] - p_prev[1], p_curr[0] - p_prev[0])
        diff = curr_heading - prev_heading
        # Normalize to [-pi, pi]
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        heading_accum += abs(math.degrees(diff))
        prev_heading = curr_heading

        # Check criteria
        dist_trigger = dist_accum >= max_spacing_cm
        heading_trigger = (
            heading_accum >= heading_threshold_deg and dist_accum >= min_spacing_cm
        )

        if dist_trigger or heading_trigger:
            waypoints.append(p_curr)
            dist_accum = 0.0
            heading_accum = 0.0

    # Always include the goal (might be duplicate if last step triggered, logic handles it)
    if waypoints[-1] != world_pts[-1]:
        waypoints.append(world_pts[-1])

    return waypoints


# =============================================================================
# Top-Level API
# =============================================================================


def plan_and_smooth(
    grid,  # OccupancyGrid instance
    start_xy: tuple,
    goal_xy: tuple,
    robot_radius_cm: float = 9.0,
    obstacle_inflation: int = 5,
    free_inflation: int = 3,
) -> list[tuple] | None:
    """Full path planning pipeline: A* → simplify → smooth → sample.

    Args:
        grid: OccupancyGrid instance.
        start_xy: (x, y) start position in world cm.
        goal_xy: (x, y) goal position in world cm.
        robot_radius_cm: Half-width of robot for obstacle clearance.
        obstacle_inflation: Cells to inflate obstacles by.
        free_inflation: Cells to inflate free space by.

    Returns:
        List of (x, y) world-coordinate waypoints, or None if no path.
    """
    # Step 1: Get traversability grid (you'll implement this in occupancy_grid.py)
    traversable = grid.get_traversability_grid(obstacle_inflation, free_inflation)

    # Step 2: Convert start/goal to grid coordinates
    start_rc = grid.world_to_grid(*start_xy)
    goal_rc = grid.world_to_grid(*goal_xy)

    # Step 3: A* search
    raw_path = a_star(traversable, start_rc, goal_rc)
    if raw_path is None:
        return None

    # Step 4: Simplify (remove redundant waypoints)
    simplified = simplify_path(raw_path, traversable)

    # Step 4b: Interpolate (add points back for smoothing)
    interpolated = _interpolate_path(simplified)

    # Step 5: Smooth (gradient descent)
    smoothed = smooth_path(interpolated)

    # Step 6: Adaptive resampling → world coordinates
    waypoints = sample_waypoints(smoothed, grid.grid_to_world)

    return waypoints


def _interpolate_path(path: list[tuple], max_spacing: float = 4.0) -> list[tuple]:
    """Add intermediate points to simplified path segments.

    Ensures gradient descent has enough points to create smooth curves.
    """
    if len(path) < 2:
        return path

    new_path = []
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        num_segments = math.ceil(dist / max_spacing)

        # Add points (excluding the last one, which is start of next segment)
        for j in range(num_segments):
            alpha = j / num_segments
            r = p0[0] * (1 - alpha) + p1[0] * alpha
            c = p0[1] * (1 - alpha) + p1[1] * alpha
            new_path.append((r, c))

    new_path.append(path[-1])
    return new_path
