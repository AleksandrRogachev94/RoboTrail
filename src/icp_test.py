import math

import numpy as np
from matplotlib import pyplot as plt

from icp import icp

# Room dimensions (meters)
room_width = 2.5  # 2.5 meters = typical small room width
room_height = 3.0  # 3 meters = typical small room length
# # Point spacing along walls (meters)
point_spacing = 0.1  # 3cm between points (denser than sensor noise)
# # Sensor characteristics
# max_sensor_range = 2.0  # VL53L1X typically 0.4-4m, use conservative 2m
sensor_fov = 180  # degrees (your servo sweeps 180°)
# # Typical robot movements to test
# test_translation = [0.15, 0.08]  # 15cm forward, 8cm sideways
# test_rotation = 10  # degrees
# # Noise level
# sensor_noise_std = 0.02  # 2cm standard deviation (VL53L1X spec)


# 1. Map generation functions
def generate_rectangle_map(width, height, point_spacing):
    """
    Generate points along rectangle perimeter.

    Args:
        width: Room width in meters (e.g., 2.5)
        height: Room height in meters (e.g., 3.0)
        point_spacing: Distance between points in meters (e.g., 0.03)

    Returns:
        numpy array of shape (N, 2) with x, y coordinates in meters
    """
    points = []

    # Bottom wall: (0, 0) to (width, 0)
    num_points = int(width / point_spacing)
    for i in range(num_points):
        x = i * point_spacing
        points.append([x, 0.0])

    # Right wall: (width, 0) to (width, height)
    num_points = int(height / point_spacing)
    for i in range(num_points):
        y = i * point_spacing
        points.append([width, y])

    # Top wall: (width, height) to (0, height)
    num_points = int(width / point_spacing)
    for i in range(num_points):
        x = width - i * point_spacing
        points.append([x, height])

    # Left wall: (0, height) to (0, 0)
    num_points = int(height / point_spacing)
    for i in range(num_points):
        y = height - i * point_spacing
        points.append([0.0, y])

    return np.array(points)


def sample_scan_from_map(map_points, num_samples=20, seed=42):
    """
    Create test scan by sampling a subset of map

    Args:
        map_points: Full map (N, 2)
        num_samples: Number of points to sample
        seed: Random seed for reproducibility

    Returns:
        scan_points, true_R, true_t
    """
    # Set seed for deterministic sampling
    np.random.seed(seed)

    # 1. Sample from right half of room (simulate partial FOV)
    section_mask = map_points[:, 0] > room_width * 0.5  # right 50% of room
    section = map_points[section_mask]

    # 2. Randomly sample subset from this section
    indices = np.random.choice(len(section), num_samples, replace=False)
    scan = section[indices].copy()

    # 2. Apply known transformation
    theta = math.radians(2)  # 10 degrees
    R = np.array(
        [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
    )
    t = np.array([0.04, 0.08])  # 15cm, 8cm

    scan_transformed = (R @ scan.T).T + t

    # 3. Add noise
    noise = np.random.normal(0, 0.02, scan_transformed.shape)
    scan_noisy = scan_transformed + noise

    return scan_noisy, R, t


def visualize_iteration(map_points, scan_original, scan_current, iteration):
    plt.figure(figsize=(8, 6))

    # Plot map in blue
    plt.scatter(
        map_points[:, 0], map_points[:, 1], c="blue", s=20, alpha=0.5, label="Map"
    )

    # Plot original scan in red
    plt.scatter(
        scan_original[:, 0],
        scan_original[:, 1],
        c="red",
        s=30,
        alpha=0.6,
        label="Scan (original)",
    )

    # # Plot current alignment in green
    plt.scatter(
        scan_current[:, 0],
        scan_current[:, 1],
        c="green",
        s=30,
        label=f"Scan (iter {iteration})",
    )

    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title(f"ICP Iteration {iteration}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.pause(0.2)  # Pause to see progress
    # plt.show()


if __name__ == "__main__":
    map_points = generate_rectangle_map(room_width, room_height, point_spacing)
    scan, true_R, true_t = sample_scan_from_map(map_points)

    estimated_R, estimated_t, aligned_scan, converged = icp(scan, map_points)
    print(f"Converged: {converged}")

    visualize_iteration(map_points, scan, aligned_scan, "Final")
    plt.show()
