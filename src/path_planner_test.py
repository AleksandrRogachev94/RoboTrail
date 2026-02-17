"""Test path planning pipeline with a clean synthetic grid.

Directly draws walls on the occupancy grid (no simulated scans)
for clear, predictable test cases.

Run: cd /path/to/robot/src && python path_planner_test.py
"""

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

from occupancy_grid import OccupancyGrid
from path_planner import (
    _interpolate_path,
    a_star,
    sample_waypoints,
    simplify_path,
    smooth_path,
)


def build_clean_grid():
    """Create a clean occupancy grid by directly setting cell values.

    Room layout (300x300 cm):

    Start at bottom-left, Goal at top-right.
    Two obstacles force an S-curve path.
    """
    grid = OccupancyGrid()

    # Mark wide area as free
    r_min, c_min = grid.world_to_grid(-50, -50)
    r_max, c_max = grid.world_to_grid(250, 250)
    grid.grid[
        min(r_min, r_max) : max(r_min, r_max) + 1,
        min(c_min, c_max) : max(c_min, c_max) + 1,
    ] = -3.0

    # Draw walls
    def draw_wall(x0, y0, x1, y1, thickness=2):
        r0, c0 = grid.world_to_grid(x0, y0)
        r1, c1 = grid.world_to_grid(x1, y1)
        r_lo, r_hi = min(r0, r1), max(r0, r1)
        c_lo, c_hi = min(c0, c1), max(c0, c1)
        grid.grid[
            r_lo - thickness : r_hi + thickness + 1,
            c_lo - thickness : c_hi + thickness + 1,
        ] = 4.0

    # Boundary box
    draw_wall(-50, -50, 250, -50)
    draw_wall(-50, 250, 250, 250)
    draw_wall(-50, -50, -50, 250)
    draw_wall(250, -50, 250, 250)

    # Obstacle 1: Vertical wall from bottom
    draw_wall(80, -50, 80, 120, thickness=6)

    # Obstacle 2: Vertical wall from top
    draw_wall(160, 50, 160, 250, thickness=6)

    # Obstacle 3: Small island
    draw_wall(40, 180, 60, 180, thickness=6)

    return grid


def plot_results(
    grid,
    start_xy,
    goal_xy,
    raw_path=None,
    simplified=None,
    interpolated=None,
    smoothed=None,
    waypoints=None,
    save_path="test_path_planner.png",
):
    """Visualize the path planning pipeline."""
    prob_map = grid.get_probability_map()
    r0, r1, c0, c1 = grid._get_data_bounds(padding=5)
    x_min, y_min = grid.grid_to_world(r0, c0)
    x_max, y_max = grid.grid_to_world(r1, c1)
    extent = [x_min, x_max, y_min, y_max]
    cropped = prob_map[r0 : r1 + 1, c0 : c1 + 1]

    # Get traversability
    try:
        traversable = grid.get_traversability_grid()
        trav_cropped = traversable[r0 : r1 + 1, c0 : c1 + 1].astype(float)
    except NotImplementedError:
        trav_cropped = None

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    fig.suptitle("Path Planning Pipeline", fontsize=14, fontweight="bold")

    titles = [
        "1. Traversability + A*",
        "2. Simplified",
        "3. Smoothed",
        "4. Final Waypoints",
    ]

    blue_cmap = mcolors.ListedColormap(["none", "#3498db40"])

    for ax, title in zip(axes.flat, titles):
        ax.imshow(cropped, cmap="gray_r", origin="lower", extent=extent, vmin=0, vmax=1)
        if trav_cropped is not None:
            ax.imshow(
                trav_cropped,
                cmap=blue_cmap,
                origin="lower",
                extent=extent,
                vmin=0,
                vmax=1,
            )
        ax.plot(*start_xy, "go", markersize=10, zorder=5)
        ax.plot(*goal_xy, "r*", markersize=12, zorder=5)
        ax.set_title(title)
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.set_aspect("equal")

    if raw_path:
        pts = [grid.grid_to_world(r, c) for r, c in raw_path]
        axes[0, 0].plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            "b-",
            lw=1.5,
            alpha=0.8,
            label=f"A* ({len(raw_path)} pts)",
        )
        axes[0, 0].legend(fontsize=8)

    if simplified:
        pts = [grid.grid_to_world(r, c) for r, c in simplified]
        axes[0, 1].plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            "m-o",
            lw=2,
            ms=6,
            label=f"Simplified ({len(simplified)} pts)",
        )
        if interpolated:
            ipts = [grid.grid_to_world(r, c) for r, c in interpolated]
            axes[0, 1].plot(
                [p[0] for p in ipts],
                [p[1] for p in ipts],
                ".",
                color="orange",
                ms=3,
                alpha=0.8,
                label=f"Interpolated ({len(interpolated)} pts)",
            )
        axes[0, 1].legend(fontsize=8)

    if smoothed:
        pts = [grid.grid_to_world(r, c) for r, c in smoothed]
        axes[1, 0].plot(
            [p[0] for p in pts],
            [p[1] for p in pts],
            "c-",
            lw=2,
            label=f"Smoothed ({len(smoothed)} pts)",
        )
        axes[1, 0].legend(fontsize=8)

    if waypoints:
        axes[1, 1].plot(
            [p[0] for p in waypoints],
            [p[1] for p in waypoints],
            "r-o",
            lw=2,
            ms=8,
            label=f"Waypoints ({len(waypoints)} pts)",
        )
        axes[1, 1].legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved {save_path}")
    plt.close()


if __name__ == "__main__":
    print("Building clean test grid...")
    grid = build_clean_grid()

    start_xy = (0, 0)
    goal_xy = (220, 200)

    print(f"Planning from {start_xy} to {goal_xy}\n")

    # Stage 1: Traversability
    print("── Stage 1: Traversability Grid ──")
    traversable = None
    try:
        traversable = grid.get_traversability_grid()
        start_rc = grid.world_to_grid(*start_xy)
        goal_rc = grid.world_to_grid(*goal_xy)
        n = np.sum(traversable)
        print(f"  {n} traversable cells ({100 * n / traversable.size:.1f}%)")
        print(f"  Start: {'✓' if traversable[start_rc] else '✗ BLOCKED'}")
        print(f"  Goal:  {'✓' if traversable[goal_rc] else '✗ BLOCKED'}")
    except NotImplementedError:
        print("  ⚠ Not implemented")

    # Stage 2: A*
    print("\n── Stage 2: A* Search ──")
    raw_path = None
    if traversable is not None:
        try:
            raw_path = a_star(traversable, start_rc, goal_rc)
            if raw_path:
                print(f"  ✓ {len(raw_path)} cells")
            else:
                print("  ✗ No path found")
        except NotImplementedError:
            print("  ⚠ Not implemented")

    # Stage 3: Simplify
    print("\n── Stage 3: Simplification ──")
    simplified = None
    if raw_path:
        try:
            simplified = simplify_path(raw_path, traversable)
            print(f"  ✓ {len(raw_path)} → {len(simplified)} waypoints")
        except NotImplementedError:
            print("  ⚠ Not implemented")

    # Stage 3b: Interpolate
    print("\n── Stage 3b: Interpolation ──")
    interpolated = None
    if simplified:
        interpolated = _interpolate_path(simplified)
        print(f"  ✓ {len(simplified)} → {len(interpolated)} points")

    # Stage 4: Smooth
    print("\n── Stage 4: Smoothing ──")
    smoothed = None
    if interpolated:
        try:
            smoothed = smooth_path(interpolated)
            print(f"  ✓ {len(smoothed)} points")
        except NotImplementedError:
            print("  ⚠ Not implemented")

    # Stage 5: Resample
    print("\n── Stage 5: Waypoints ──")
    waypoints = None
    if smoothed:
        try:
            waypoints = sample_waypoints(smoothed, grid.grid_to_world)
            print(f"  ✓ {len(waypoints)} waypoints:")
            for i, (x, y) in enumerate(waypoints):
                print(f"    [{i}] ({x:.1f}, {y:.1f})")
        except NotImplementedError:
            print("  ⚠ Not implemented")

    # Plot
    print()
    plot_results(
        grid, start_xy, goal_xy, raw_path, simplified, interpolated, smoothed, waypoints
    )
