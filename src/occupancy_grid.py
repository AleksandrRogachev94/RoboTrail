"""Probabilistic occupancy grid using log-odds representation.

Each cell in the grid stores a log-odds value:
  l = log(P(occupied) / P(free))

  l = 0   → 50% probability (unknown)
  l > 0   → more likely occupied
  l < 0   → more likely free

Updating is simple addition:
  - Ray passes through cell → subtract L_FREE (evidence it's free)
  - Ray hits cell           → add L_OCC   (evidence it's occupied)
"""

import math

import matplotlib.pyplot as plt
import numpy as np

from robot.config import (
    GRID_ORIGIN,
    GRID_RESOLUTION,
    GRID_SIZE,
    L_FREE,
    L_MAX,
    L_MIN,
    L_OCC,
    TOF_OFFSET_X,
    TOF_OFFSET_Y,
)


class OccupancyGrid:
    """2D probabilistic occupancy grid."""

    def __init__(self):
        """Initialize grid as all unknown (log-odds = 0)."""
        self.grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    # ── Coordinate Conversion ──────────────────────────────────────────

    def world_to_grid(self, world_x: float, world_y: float) -> tuple[int, int]:
        """Convert world coordinates (cm) to grid indices (row, col).

        World frame: X = forward, Y = left (matches robot frame).
        Grid frame:  row = Y axis, col = X axis.

        Args:
            world_x: X position in cm.
            world_y: Y position in cm.

        Returns:
            (row, col) grid indices.
        """
        col = GRID_ORIGIN + int(math.floor(world_x / GRID_RESOLUTION))
        row = GRID_ORIGIN + int(math.floor(world_y / GRID_RESOLUTION))
        return row, col

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid indices back to world coordinates (cm).

        Args:
            row: Grid row index.
            col: Grid column index.

        Returns:
            (world_x, world_y) in cm.
        """
        world_x = (col - GRID_ORIGIN) * GRID_RESOLUTION
        world_y = (row - GRID_ORIGIN) * GRID_RESOLUTION
        return world_x, world_y

    # ── Ray Tracing ────────────────────────────────────────────────────

    def bresenham(self, r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1  # Fixed typo from your snippet

        line = []
        r, c = r0, c0

        if dc >= dr:
            # Column is the "fast" axis
            error = dc // 2
            while c != c1 + sc:  # Iterate until we pass the endpoint
                line.append((r, c))
                c += sc
                error -= dr
                if error < 0:
                    r += sr
                    error += dc
        else:
            # Row is the "fast" axis
            error = dr // 2
            while r != r1 + sr:
                line.append((r, c))
                r += sr
                error -= dc
                if error < 0:
                    c += sc
                    error += dr

        return line

    # ── Map Update ─────────────────────────────────────────────────────

    def update(self, scan_points: np.ndarray, robot_pose: tuple) -> None:
        """Update the grid with one scan from a known robot pose.

        For each scan point:
        1. Compute the sensor position in world frame (apply TOF offset)
        2. Convert sensor position and hit point to grid coordinates
        3. Bresenham ray trace from sensor to hit point
        4. Mark free cells (all cells along ray EXCEPT last) → subtract L_FREE
        5. Mark occupied cell (last cell in ray) → add L_OCC
        6. Clamp all updated cells to [L_MIN, L_MAX]

        Args:
            scan_points: (N, 2) array of hit points in ROBOT frame (from Scanner).
                         Each row is [x, y] in cm relative to robot center.
            robot_pose:  (x, y, heading_deg) — robot position in world frame.
                         x, y in cm, heading in degrees.
        """
        rx, ry, heading_deg = robot_pose
        heading = math.radians(heading_deg)

        # Sensor position in world frame (apply TOF offset)
        # The TOF is mounted TOF_OFFSET_X cm forward of robot center
        sensor_wx = (
            rx + TOF_OFFSET_X * math.cos(heading) - TOF_OFFSET_Y * math.sin(heading)
        )
        sensor_wy = (
            ry + TOF_OFFSET_X * math.sin(heading) + TOF_OFFSET_Y * math.cos(heading)
        )

        # Convert sensor position to grid
        sensor_row, sensor_col = self.world_to_grid(sensor_wx, sensor_wy)

        for point in scan_points:
            local_x, local_y = point

            # Robot frame → world frame (rotate by heading, translate by robot pos)
            world_x = rx + local_x * math.cos(heading) - local_y * math.sin(heading)
            world_y = ry + local_x * math.sin(heading) + local_y * math.cos(heading)

            # Convert hit point to grid coordinates
            hit_row, hit_col = self.world_to_grid(world_x, world_y)

            # Bounds check
            if not (0 <= hit_row < GRID_SIZE and 0 <= hit_col < GRID_SIZE):
                continue

            # Bresenham ray from sensor to hit point
            ray = self.bresenham(sensor_row, sensor_col, hit_row, hit_col)

            # Mark free cells (all except last)
            for r, c in ray[:-1]:
                if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
                    self.grid[r, c] -= L_FREE

            # Mark occupied cell (last cell)
            r, c = ray[-1]
            if 0 <= r < GRID_SIZE and 0 <= c < GRID_SIZE:
                self.grid[r, c] += L_OCC

        # Clamp entire grid
        np.clip(self.grid, L_MIN, L_MAX, out=self.grid)

    # ── Queries ─────────────────────────────────────────────────────────

    def get_probability(self, row: int, col: int) -> float:
        """Convert a single cell's log-odds to probability.

        Formula: p = 1 / (1 + exp(-l))

        Args:
            row, col: Grid cell indices.

        Returns:
            Probability of occupancy [0.0 to 1.0].
        """
        return 1 / (1 + math.exp(-self.grid[row, col]))

    def get_probability_map(self) -> np.ndarray:
        """Convert entire grid from log-odds to probability.

        Returns:
            (GRID_SIZE, GRID_SIZE) array with values in [0.0, 1.0].
        """
        return 1 / (1 + np.exp(-self.grid))

    def is_occupied(self, row: int, col: int, threshold: float = 0.7) -> bool:
        """Check if a cell is considered occupied.

        Args:
            row, col: Grid cell indices.
            threshold: Probability above which cell is "occupied".

        Returns:
            True if P(occupied) > threshold.
        """
        return self.get_probability(row, col) > threshold

    def is_free(self, row: int, col: int, threshold: float = 0.3) -> bool:
        """Check if a cell is considered free.

        Args:
            row, col: Grid cell indices.
            threshold: Probability below which cell is "free".

        Returns:
            True if P(occupied) < threshold.
        """
        return self.get_probability(row, col) < threshold

    # ── Visualization ──────────────────────────────────────────────────

    def _get_data_bounds(self, padding: int = 10) -> tuple[int, int, int, int]:
        """Find the bounding box of cells that have been updated (non-zero).

        Only shows the region with actual data, plus padding.

        Args:
            padding: Extra cells to show around the data bounds.

        Returns:
            (row_min, row_max, col_min, col_max) — cropped view bounds.
        """
        nonzero = np.nonzero(self.grid)
        if len(nonzero[0]) == 0:
            # No data yet — return center region
            c = GRID_ORIGIN
            return c - padding, c + padding, c - padding, c + padding

        row_min = max(0, nonzero[0].min() - padding)
        row_max = min(GRID_SIZE - 1, nonzero[0].max() + padding)
        col_min = max(0, nonzero[1].min() - padding)
        col_max = min(GRID_SIZE - 1, nonzero[1].max() + padding)
        return row_min, row_max, col_min, col_max

    def plot(
        self, robot_pose: tuple | None = None, save_path: str | None = None
    ) -> None:
        """Visualize the occupancy grid with optional robot pose.

        - Unknown cells (log-odds ≈ 0) → gray
        - Free cells (log-odds < 0) → white
        - Occupied cells (log-odds > 0) → black
        - Robot → arrow showing position and heading

        The plot auto-crops to the region with data.

        Args:
            robot_pose: Optional (x, y, heading_deg) to draw robot arrow.
            save_path: Optional path to save the figure as an image.
        """
        prob_map = self.get_probability_map()
        row_min, row_max, col_min, col_max = self._get_data_bounds()

        # Crop to data region
        cropped = prob_map[row_min : row_max + 1, col_min : col_max + 1]

        # Convert grid bounds to world coordinates for axis labels
        x_min, y_min = self.grid_to_world(row_min, col_min)
        x_max, y_max = self.grid_to_world(row_max, col_max)

        fig, ax = plt.subplots(1, 1, figsize=(10, 8))

        # imshow: 0.0 (free) → white, 0.5 (unknown) → gray, 1.0 (occupied) → black
        im = ax.imshow(
            cropped,
            cmap="gray_r",
            origin="lower",
            extent=[x_min, x_max, y_min, y_max],
            vmin=0,
            vmax=1,
        )

        # Draw robot pose as arrow
        if robot_pose is not None:
            rx, ry, heading_deg = robot_pose
            heading = math.radians(heading_deg)
            arrow_len = 8  # cm
            ax.annotate(
                "",
                xy=(
                    rx + arrow_len * math.cos(heading),
                    ry + arrow_len * math.sin(heading),
                ),
                xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="red", lw=2),
            )
            ax.plot(rx, ry, "ro", markersize=6)

        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.set_title("Occupancy Grid")
        ax.set_aspect("equal")
        fig.colorbar(im, ax=ax, label="P(occupied)", shrink=0.8)

        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Saved to {save_path}")
        else:
            plt.show()
