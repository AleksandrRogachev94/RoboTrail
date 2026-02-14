"""Test occupancy grid with synthetic scan data.

Creates a simple rectangular room, simulates a scan from the center,
updates the grid, and visualizes the result.

Run: cd /path/to/robot/src && python occupancy_grid_test.py
"""

import math

import numpy as np

from occupancy_grid import OccupancyGrid


def simulate_scan(robot_x, robot_y, heading_deg, walls, num_rays=19, fov=120):
    """Simulate a ToF scan by ray-casting against known walls.

    Generates scan points in ROBOT frame (same format as Scanner.scan()).

    Args:
        robot_x, robot_y: Robot position in world frame (cm).
        heading_deg: Robot heading in degrees.
        walls: List of wall segments [(x1,y1,x2,y2), ...] in world frame (cm).
        num_rays: Number of scan rays.
        fov: Field of view in degrees (centered on heading).

    Returns:
        (N, 2) numpy array of hit points in ROBOT frame [x, y] in cm.
        Points where no wall was hit are excluded.
    """
    heading = math.radians(heading_deg)
    half_fov = fov / 2
    angles = np.linspace(-half_fov, half_fov, num_rays)

    points = []
    max_range = 400  # cm (4m max sensor range)

    for angle_deg in angles:
        # Ray direction in world frame
        ray_angle = heading + math.radians(angle_deg)
        ray_dx = math.cos(ray_angle)
        ray_dy = math.sin(ray_angle)

        # Find closest wall intersection
        closest_dist = max_range
        for x1, y1, x2, y2 in walls:
            # Line-segment intersection (ray vs wall segment)
            dist = _ray_segment_intersection(
                robot_x, robot_y, ray_dx, ray_dy, x1, y1, x2, y2
            )
            if dist is not None and dist < closest_dist:
                closest_dist = dist

        if closest_dist < max_range:
            # Convert to robot frame (angle_deg relative to heading)
            local_x = closest_dist * math.cos(math.radians(angle_deg))
            local_y = -closest_dist * math.sin(math.radians(angle_deg))
            points.append([local_x, local_y])

    return np.array(points)


def _ray_segment_intersection(ox, oy, dx, dy, x1, y1, x2, y2):
    """Find intersection distance of ray (origin, direction) with segment.

    Returns distance along ray, or None if no intersection.
    """
    # Segment direction
    sx, sy = x2 - x1, y2 - y1

    denom = dx * sy - dy * sx
    if abs(denom) < 1e-10:
        return None  # Parallel

    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom  # Ray parameter
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom  # Segment parameter

    if t > 0 and 0 <= u <= 1:
        return t
    return None


def make_room_walls(width_cm, height_cm, offset_x=0, offset_y=0):
    """Create wall segments for a rectangular room.

    Args:
        width_cm: Room width in cm.
        height_cm: Room height in cm.
        offset_x, offset_y: Bottom-left corner position.

    Returns:
        List of (x1, y1, x2, y2) wall segments.
    """
    x0, y0 = offset_x, offset_y
    x1, y1 = offset_x + width_cm, offset_y + height_cm
    return [
        (x0, y0, x1, y0),  # Bottom wall
        (x1, y0, x1, y1),  # Right wall
        (x1, y1, x0, y1),  # Top wall
        (x0, y1, x0, y0),  # Left wall
    ]


if __name__ == "__main__":
    # ── Setup ──────────────────────────────────────────────────────
    # Room: 250cm × 300cm, robot starts near center
    walls = make_room_walls(250, 300, offset_x=-125, offset_y=-50)

    robot_pose = (0, 100, 0)  # (x=0cm, y=100cm, heading=0° = facing +X)

    # ── Simulate scan ──────────────────────────────────────────────
    scan = simulate_scan(
        robot_pose[0], robot_pose[1], robot_pose[2], walls, num_rays=19, fov=120
    )
    print(f"Simulated {len(scan)} scan points")

    # ── Update grid ────────────────────────────────────────────────
    grid = OccupancyGrid()
    grid.update(scan, robot_pose)

    # ── Verify ─────────────────────────────────────────────────────
    # Check that some cells became occupied (positive log-odds)
    num_occupied = np.sum(grid.grid > 0)
    num_free = np.sum(grid.grid < 0)
    print(f"Occupied cells: {num_occupied}")
    print(f"Free cells:     {num_free}")
    print(f"Max log-odds:   {grid.grid.max():.2f}")
    print(f"Min log-odds:   {grid.grid.min():.2f}")

    assert num_occupied > 0, "No occupied cells! update() isn't marking walls."
    assert num_free > 0, "No free cells! Bresenham rays aren't marking free space."
    print("\n✓ Basic checks passed!")

    # ── Visualize ──────────────────────────────────────────────────
    grid.plot(robot_pose=robot_pose, save_path="test_occupancy_grid.png")
    print("Saved test_occupancy_grid.png")
