import lgpio
import matplotlib.pyplot as plt
import numpy as np

from icp import icp
from robot.drive import Robot
from scanner import Scanner

if __name__ == "__main__":
    chip = lgpio.gpiochip_open(4)
    robot = Robot(chip)
    scanner = Scanner()
    move_cm = 30
    rotate_deg = -20
    # Step 1: Reference scan
    print("Taking reference scan (Scan A)...")
    scan_a = scanner.scan()
    print(f"  Scan A: {len(scan_a)} points")
    # Step 2: Move robot
    print(f"Moving forward {move_cm} cm...")
    robot.forward(move_cm)
    print(f"Rotating {rotate_deg} degrees...")
    robot.turn(rotate_deg)
    # Step 3: Second scan
    print("Taking second scan (Scan B)...")
    scan_b = scanner.scan()
    print(f"  Scan B: {len(scan_b)} points")
    # Step 4: Apply odometry transform (translation AND rotation)
    theta = np.radians(-rotate_deg)  # Negative of robot rotation
    R_odom = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    # First rotate, then translate
    scan_b_transformed = (R_odom @ scan_b.T).T + [0, move_cm]
    print(f"Odometry estimate: translate ({0}, {move_cm}) cm, rotate {rotate_deg}°")
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
        f"ICP Test: moved {move_cm}cm, rotated {rotate_deg}°, correction=({t[0]:.1f}, {t[1]:.1f})cm, {angle_deg:.1f}°"
    )
    # plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("test_icp_real.png", dpi=150)
    print("Saved to test_icp_real.png")
    # Cleanup
    robot.stop()
    lgpio.gpiochip_close(chip)
