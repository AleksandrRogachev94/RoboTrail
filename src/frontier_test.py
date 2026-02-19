"""Test frontier detection and goal selection with synthetic grids.

Creates a partially-explored room, detects frontiers, selects a goal,
and visualizes everything.

Run: cd /Users/alex/Documents/code/robot/src && python frontier_test.py
"""

import math
import sys

import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np

# Add src to path for imports
sys.path.insert(0, "/Users/alex/Documents/code/robot/src")

from frontier import find_frontiers, get_frontier_viz_data, select_goal
from occupancy_grid import OccupancyGrid
from path_planner import a_star


def build_partial_grid():
    """Create a grid with a partially-explored room.

    Layout (200x200 cm room):
      - Bottom-left quadrant is explored (free)
      - Top and right are unknown
      - Walls along bottom and left edges
      - Interior obstacle to test path planning around it

    This simulates what the grid looks like after a few scans from
    the origin area.
    """
    grid = OccupancyGrid()

    # Mark bottom-left area as free (explored)
    r_lo, c_lo = grid.world_to_grid(-30, -30)
    r_hi, c_hi = grid.world_to_grid(100, 100)
    r_min, r_max = min(r_lo, r_hi), max(r_lo, r_hi)
    c_min, c_max = min(c_lo, c_hi), max(c_lo, c_hi)
    grid.grid[r_min : r_max + 1, c_min : c_max + 1] = -3.0  # free

    # Add walls along the bottom and left edges
    def draw_wall(x0, y0, x1, y1, thickness=2):
        r0, c0 = grid.world_to_grid(x0, y0)
        r1, c1 = grid.world_to_grid(x1, y1)
        rl, rh = min(r0, r1), max(r0, r1)
        cl, ch = min(c0, c1), max(c0, c1)
        grid.grid[
            rl - thickness : rh + thickness + 1,
            cl - thickness : ch + thickness + 1,
        ] = 4.0  # occupied

    # Room boundary walls (only on explored edges)
    draw_wall(-30, -30, 200, -30)  # Bottom wall
    draw_wall(-30, -30, -30, 200)  # Left wall

    # Interior obstacle
    draw_wall(50, 30, 50, 70, thickness=3)

    return grid


def plot_frontiers(grid, clusters, goal, robot_pose, save_path="test_frontier.png"):
    """Visualize frontiers, goal, and traversability."""
    prob_map = grid.get_probability_map()
    r0, r1, c0, c1 = grid._get_data_bounds(padding=5)
    x_min, y_min = grid.grid_to_world(r0, c0)
    x_max, y_max = grid.grid_to_world(r1, c1)
    extent = [x_min, x_max, y_min, y_max]
    cropped = prob_map[r0 : r1 + 1, c0 : c1 + 1]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # Left: Occupancy map with frontiers
    ax = axes[0]
    ax.imshow(cropped, cmap="gray_r", origin="lower", extent=extent, vmin=0, vmax=1)
    ax.set_title(f"Frontiers ({len(clusters)} clusters)")
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_aspect("equal")

    # Draw frontier cells
    colors = plt.cm.Set1(np.linspace(0, 1, max(len(clusters), 1)))
    for i, cluster in enumerate(clusters):
        pts = [grid.grid_to_world(r, c) for r, c in cluster]
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.scatter(
            xs,
            ys,
            c=[colors[i % len(colors)]],
            s=8,
            alpha=0.7,
            label=f"Cluster {i} ({len(cluster)} cells)",
        )

    # Robot
    rx, ry, heading_deg = robot_pose
    ax.plot(rx, ry, "ro", markersize=8, zorder=5)
    heading_rad = math.radians(heading_deg)
    ax.annotate(
        "",
        xy=(rx + 10 * math.cos(heading_rad), ry + 10 * math.sin(heading_rad)),
        xytext=(rx, ry),
        arrowprops=dict(arrowstyle="->", color="red", lw=2),
    )

    # Goal
    if goal:
        ax.plot(goal[0], goal[1], "m*", markersize=15, zorder=5, label="Goal")

    ax.legend(fontsize=8, loc="upper left")

    # Right: Traversability with path to goal
    ax2 = axes[1]
    try:
        traversable = grid.get_traversability_grid()
        trav_cropped = traversable[r0 : r1 + 1, c0 : c1 + 1].astype(float)
        blue_cmap = mcolors.ListedColormap(["#1a1a2e", "#2d5a88"])
        ax2.imshow(
            trav_cropped, cmap=blue_cmap, origin="lower", extent=extent, vmin=0, vmax=1
        )
    except Exception:
        ax2.imshow(
            cropped, cmap="gray_r", origin="lower", extent=extent, vmin=0, vmax=1
        )

    ax2.set_title("Traversability + Goal")
    ax2.set_xlabel("X (cm)")
    ax2.set_ylabel("Y (cm)")
    ax2.set_aspect("equal")

    ax2.plot(rx, ry, "ro", markersize=8, zorder=5)
    if goal:
        ax2.plot(goal[0], goal[1], "m*", markersize=15, zorder=5)
        # Compute and draw actual path
        start_rc = grid.world_to_grid(rx, ry)
        goal_rc = grid.world_to_grid(goal[0], goal[1])
        try:
            traversable = grid.get_traversability_grid()
            # Ensure start is traversable for the test (simulating being safe)
            sr, sc = start_rc
            rows, cols = traversable.shape
            CLEAR_RADIUS = 5
            r_lo = max(0, sr - CLEAR_RADIUS)
            r_hi = min(rows, sr + CLEAR_RADIUS + 1)
            c_lo = max(0, sc - CLEAR_RADIUS)
            c_hi = min(cols, sc + CLEAR_RADIUS + 1)
            traversable[r_lo:r_hi, c_lo:c_hi] = True

            path = a_star(traversable, start_rc, goal_rc)
            if path:
                path_world = [grid.grid_to_world(r, c) for r, c in path]
                px = [p[0] for p in path_world]
                py = [p[1] for p in path_world]
                ax2.plot(px, py, "m-", lw=2, label="A* Path")
            else:
                print("Warning: Goal selected but no path found in re-check!")
        except Exception as e:
            print(f"Path planning failed: {e}")

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved {save_path}")
    plt.close()


if __name__ == "__main__":
    print("Building partially-explored grid...")
    grid = build_partial_grid()

    robot_pose = (20, 20, 45)  # Facing northeast
    print(f"Robot pose: {robot_pose}")

    # Find frontiers
    print("\n── Finding frontiers ──")
    clusters = find_frontiers(grid)
    print(f"Found {len(clusters)} clusters")
    for i, cluster in enumerate(clusters):
        cr = sum(r for r, c in cluster) / len(cluster)
        cc = sum(c for r, c in cluster) / len(cluster)
        cx, cy = grid.grid_to_world(cr, cc)
        print(f"  Cluster {i}: {len(cluster)} cells, centroid=({cx:.0f}, {cy:.0f})")

    # Select goal
    print("\n── Selecting goal ──")
    goal = select_goal(grid, clusters, robot_pose)
    if goal:
        dist = math.hypot(goal[0] - robot_pose[0], goal[1] - robot_pose[1])
        print(f"Goal: ({goal[0]:.1f}, {goal[1]:.1f}), dist={dist:.1f}cm")
    else:
        print("No reachable goal!")

    # Viz data (for web UI)
    print("\n── Viz data ──")
    viz = get_frontier_viz_data(grid, clusters)
    for i, v in enumerate(viz):
        print(f"  Cluster {i}: centroid={v['centroid']}, size={v['size']}")

    # Plot
    print()
    plot_frontiers(grid, clusters, goal, robot_pose, save_path="test_frontier.png")
