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
from scipy.ndimage import binary_dilation

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


def scan_to_world(scan_points: np.ndarray, robot_pose: tuple):
    """Convert sensor-frame scan points to world coordinates.

    The scanner returns points relative to the TOF sensor. This function:
    1. Computes the sensor's position in world frame (robot pos + rotated offset)
    2. Rotates scan vectors by robot heading (sensor frame → world orientation)
    3. Translates from sensor position

    Args:
        scan_points: (N, 2) array of [x, y] in sensor frame (from Scanner).
        robot_pose:  (x, y, heading_deg) in world frame.

    Returns:
        (world_points, sensor_origin):
            world_points: (N, 2) array of hit positions in world frame.
            sensor_origin: (2,) sensor position in world frame.
    """
    rx, ry, heading_deg = robot_pose
    heading = math.radians(heading_deg)
    c, s = math.cos(heading), math.sin(heading)

    sensor_origin = np.array(
        [
            rx + TOF_OFFSET_X * c - TOF_OFFSET_Y * s,
            ry + TOF_OFFSET_X * s + TOF_OFFSET_Y * c,
        ]
    )

    R = np.array([[c, -s], [s, c]])
    world_points = (R @ np.asarray(scan_points).T).T + sensor_origin

    return world_points, sensor_origin


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

    @staticmethod
    def bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1

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
        1. Compute hit position in world frame (via scan_to_world)
        2. Bresenham ray trace from sensor to hit point
        3. Mark free cells (all cells along ray EXCEPT last) → subtract L_FREE
        4. Mark occupied cell (last cell in ray) → add L_OCC
        5. Clamp all updated cells to [L_MIN, L_MAX]

        Args:
            scan_points: (N, 2) array in sensor frame (from Scanner).
            robot_pose:  (x, y, heading_deg) — robot position in world frame.
        """
        world_points, sensor_origin = scan_to_world(scan_points, robot_pose)

        sensor_row, sensor_col = self.world_to_grid(sensor_origin[0], sensor_origin[1])

        for i in range(len(world_points)):
            world_x, world_y = world_points[i]

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

    def get_occupied_points(self, threshold: float = 0.6) -> np.ndarray:
        """Return world coordinates of all occupied cells.

        Args:
            threshold: Probability above which a cell is "occupied".

        Returns:
            (N, 2) array of [world_x, world_y] for each occupied cell.
        """
        prob_map = self.get_probability_map()
        rows, cols = np.where(prob_map > threshold)
        if len(rows) == 0:
            return np.empty((0, 2))
        # Convert grid indices to world coords (vectorized)
        world_x = (cols - GRID_ORIGIN) * GRID_RESOLUTION
        world_y = (rows - GRID_ORIGIN) * GRID_RESOLUTION
        return np.column_stack([world_x, world_y])

    def is_free(self, row: int, col: int, threshold: float = 0.3) -> bool:
        """Check if a cell is considered free.

        Args:
            row, col: Grid cell indices.
            threshold: Probability below which cell is "free".

        Returns:
            True if P(occupied) < threshold.
        """
        return self.get_probability(row, col) < threshold

    # ── Traversability ─────────────────────────────────────────────────

    def get_traversability_grid(
        self, obstacle_inflation: int = 7, free_inflation: int = 5
    ) -> np.ndarray:
        """Build traversability mask with dual inflation.

        The raw grid has sparse free cells (rays only mark cells they pass
        through). This method fills gaps by:
        1. Inflating FREE cells outward to bridge gaps between sparse rays
        2. Inflating OCCUPIED cells outward for safety margin
        3. Traversable = inflated_free AND NOT inflated_obstacle

        Args:
            obstacle_inflation: Cells to dilate obstacles by (~robot half-width).
                                5 cells × 2cm = 10cm clearance.
            free_inflation: Cells to dilate free space by (bridge ray gaps).
                            3 cells × 2cm = 6cm gap filling.

        Returns:
            2D boolean array (same shape as grid). True = safe to traverse.

        """
        prob_map = self.get_probability_map()
        free_mask = prob_map < 0.3
        occ_mask = prob_map > 0.7

        # Create disk structuring elements for each inflation radius
        r = obstacle_inflation
        y, x = np.ogrid[-r : r + 1, -r : r + 1]
        occ_disk = (x * x + y * y) <= r * r

        r = free_inflation
        y, x = np.ogrid[-r : r + 1, -r : r + 1]
        free_disk = (x * x + y * y) <= r * r

        # Dilate both masks
        inflated_free = binary_dilation(free_mask, structure=free_disk)
        inflated_occ = binary_dilation(occ_mask, structure=occ_disk)

        # 6. Return inflated_free & ~inflated_occ
        return inflated_free & ~inflated_occ

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
        plt.close(fig)
