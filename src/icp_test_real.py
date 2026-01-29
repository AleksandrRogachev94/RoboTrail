import lgpio
import matplotlib.pyplot as plt
import numpy as np

from icp import icp
from robot.drive_dc import RobotDC
from scanner import Scanner

if __name__ == "__main__":
    chip = lgpio.gpiochip_open(4)
    # Use DC robot
    robot = RobotDC(chip)
    scanner = Scanner()

    # Step 1: Reference scan
    print("Taking reference scan (Scan A)...")
    scan_a = scanner.scan()
    print(f"  Scan A: {len(scan_a)} points")

    # Step 2: Move robot (using arc for more realistic test)
    # We want to move ~30cm and rotate ~20 degrees
    # Arc of 85cm radius and 30cm length gives ~20 degree turn
    radius_cm = 85
    arc_length_cm = 30

    print(f"Driving arc: radius={radius_cm}cm, length={arc_length_cm}cm...")
    robot.arc(radius_cm, arc_length_cm)

    # Get actual calculated pose from odometry
    odom_x, odom_y, odom_heading = robot.get_pose()
    print(
        f"Odometry pose: x={odom_x:.1f}cm, y={odom_y:.1f}cm, heading={odom_heading:.1f}°"
    )

    # Step 3: Second scan
    print("Taking second scan (Scan B)...")
    scan_b = scanner.scan()
    print(f"  Scan B: {len(scan_b)} points")

    # Step 4: Apply odometry transform
    # Convert odometry pose to transformation matrix
    # Note: Scanning happens in robot frame, so we transform Scan B by the robot's movement
    theta = np.radians(odom_heading)
    c, s = np.cos(theta), np.sin(theta)
    R_odom = np.array([[c, -s], [s, c]])

    # Scan B points are in new robot frame. Transform them to world frame (Scan A frame)
    # P_world = R * P_robot + T
    scan_b_transformed = (R_odom @ scan_b.T).T + [odom_x, odom_y]
    # Step 5: Run ICP
    print("Running ICP...")
    R, t, aligned, converged = icp(scan_b_transformed, scan_a, max_distance=20)

    # Extract rotation angle
    angle_rad = np.arctan2(R[1, 0], R[0, 0])
    angle_deg = np.degrees(angle_rad)

    print(f"  Converged: {converged}")
    print(f"  ICP correction: t = ({t[0]:.2f}, {t[1]:.2f}) cm")
    print(f"  ICP rotation: {angle_deg:.2f}°")
    # Step 6: Apply correction
    scan_b_corrected = (R @ scan_b_transformed.T).T + t
    # Step 7: Visualize
    plt.figure(figsize=(10, 8))

    plt.scatter(
        scan_a[:, 0],
        scan_a[:, 1],
        c="blue",
        s=50,
        label="Scan A (reference)",
        alpha=0.7,
    )
    plt.scatter(
        scan_b_transformed[:, 0],
        scan_b_transformed[:, 1],
        c="red",
        s=50,
        label="Scan B (odometry)",
        alpha=0.7,
    )
    plt.scatter(
        scan_b_corrected[:, 0],
        scan_b_corrected[:, 1],
        c="green",
        s=50,
        label="Scan B (ICP corrected)",
        alpha=0.7,
    )

    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.legend()
    plt.title(
        f"ICP Test: moved ({odom_x:.1f}, {odom_y:.1f})cm, {odom_heading:.1f}°, correction=({t[0]:.1f}, {t[1]:.1f})cm, {angle_deg:.1f}°"
    )
    # plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("test_icp_real.png", dpi=150)
    print("Saved to test_icp_real.png")
    # Cleanup
    robot.stop()
    lgpio.gpiochip_close(chip)
