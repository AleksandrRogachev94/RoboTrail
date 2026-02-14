#!/usr/bin/env python3
"""End-to-end SLAM test with predefined movements.

Runs a simple stop-and-scan loop:
  1. Move (arc motions only — no point turns)
  2. Stop & scan
  3. ICP correction (optional)
  4. Update occupancy grid
  5. Save all outputs to images

Run on Pi: python3 slam_test.py
         python3 slam_test.py --no-icp
"""

import math
import os
import shutil
import sys

import lgpio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from icp import icp
from occupancy_grid import OccupancyGrid
from robot.config import TOF_OFFSET_X, TOF_OFFSET_Y
from robot.drive_dc import RobotDC
from scanner import Scanner

# ── Config ─────────────────────────────────────────────────────────────
OUTPUT_DIR = "slam_test_output"
USE_ICP = "--no-icp" not in sys.argv


def setup_output_dir():
    """Re-create output folder for clean run."""
    if os.path.exists(OUTPUT_DIR):
        shutil.rmtree(OUTPUT_DIR)
    os.makedirs(OUTPUT_DIR)
    print(f"Output directory: {OUTPUT_DIR}/")


# ── Scan helpers ───────────────────────────────────────────────────────


def scan_to_world(scan_sensor, pose):
    """Transform sensor-frame scan to world frame.

    Args:
        scan_sensor: (N, 2) points from Scanner (robot frame, no TOF offset).
        pose: (x, y, heading_deg) robot pose in world frame.

    Returns:
        (N, 2) points in world frame.
    """
    x, y, heading_deg = pose
    theta = math.radians(heading_deg)
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])

    # Add TOF offset to get robot-frame points, then rotate+translate to world
    scan_robot = scan_sensor + [TOF_OFFSET_X, TOF_OFFSET_Y]
    return (R @ scan_robot.T).T + np.array([x, y])


def save_icp_plot(
    prev_scan, curr_odom, curr_corrected, step, odom_pose, corrected_pose, converged
):
    """Save ICP alignment visualization."""
    fig, ax = plt.subplots(figsize=(10, 8))

    ax.scatter(
        prev_scan[:, 0],
        prev_scan[:, 1],
        c="blue",
        s=40,
        alpha=0.6,
        label="Previous scan",
    )
    ax.scatter(
        curr_odom[:, 0],
        curr_odom[:, 1],
        c="red",
        s=40,
        alpha=0.6,
        label="Current (odometry)",
    )
    ax.scatter(
        curr_corrected[:, 0],
        curr_corrected[:, 1],
        c="green",
        s=40,
        alpha=0.6,
        label="Current (ICP corrected)",
    )

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ox, oy, oh = odom_pose
    cx, cy, ch = corrected_pose
    ax.set_title(
        f"Step {step} ICP — converged={converged}\n"
        f"Odom: ({ox:.1f}, {oy:.1f}, {oh:.1f}°) → "
        f"Corrected: ({cx:.1f}, {cy:.1f}, {ch:.1f}°)"
    )
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    path = os.path.join(OUTPUT_DIR, f"step{step:02d}_icp.png")
    plt.savefig(path, dpi=150)
    plt.close()
    print(f"  Saved {path}")


def main():
    setup_output_dir()

    # ── Predefined movement sequence ──────────────────────────────
    # ("forward", cm) or ("arc", radius_cm, arc_length_cm)
    # Positive radius = left turn, negative = right turn
    movements = [
        ("forward", 30),
        ("arc", 40, 30),
        ("forward", 30),
        ("arc", -40, 30),
        ("forward", 30),
    ]

    # ── Initialize hardware ───────────────────────────────────────
    print("Initializing hardware...")
    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)
    scanner = Scanner()
    grid = OccupancyGrid()

    prev_scan_world = None  # For ICP

    print(f"ICP: {'ON' if USE_ICP else 'OFF'}")

    try:
        # ── Initial scan ──────────────────────────────────────────
        input("\nPress Enter to take initial scan (step 0)...")
        pose = robot.get_pose()
        scan = scanner.scan()

        grid.update(scan, pose)
        grid.plot(robot_pose=pose, save_path=os.path.join(OUTPUT_DIR, "step00_map.png"))

        if USE_ICP:
            prev_scan_world = scan_to_world(scan, pose)

        print(f"  Pose: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}°)")

        # ── Movement loop ─────────────────────────────────────────
        for step, movement in enumerate(movements, start=1):
            input(f"\nPress Enter to execute step {step}: {movement}...")

            # Execute movement
            if movement[0] == "forward":
                robot.forward(movement[1])
            elif movement[0] == "arc":
                robot.arc(movement[1], movement[2])

            pose = robot.get_pose()
            print(f"  Odometry: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}°)")

            # Scan
            print("  Scanning...")
            scan = scanner.scan()

            # ICP correction
            if USE_ICP and prev_scan_world is not None and len(scan) > 5:
                scan_world_odom = scan_to_world(scan, pose)

                print("  Running ICP...")
                R_icp, t_icp, aligned, converged = icp(
                    scan_world_odom, prev_scan_world, max_distance=20
                )

                odom_pose = pose
                if converged:
                    angle_corr = math.degrees(math.atan2(R_icp[1, 0], R_icp[0, 0]))
                    corrected_pos = R_icp @ np.array([pose[0], pose[1]]) + t_icp

                    # Update robot's internal pose with correction
                    robot.x = corrected_pos[0]
                    robot.y = corrected_pos[1]
                    robot._heading += angle_corr

                    pose = robot.get_pose()
                    print(
                        f"  ICP corrected: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}°)"
                    )
                else:
                    print("  ICP did not converge, using odometry")

                save_icp_plot(
                    prev_scan_world,
                    scan_world_odom,
                    aligned,
                    step,
                    odom_pose,
                    pose,
                    converged,
                )

            # Update map
            grid.update(scan, pose)
            grid.plot(
                robot_pose=pose,
                save_path=os.path.join(OUTPUT_DIR, f"step{step:02d}_map.png"),
            )

            if USE_ICP:
                prev_scan_world = scan_to_world(scan, pose)

        # ── Summary ───────────────────────────────────────────────
        print("\n=== Done! ===")
        print(f"Final pose: ({pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}°)")
        print(f"Output saved to {OUTPUT_DIR}/")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        robot.close()
        lgpio.gpiochip_close(chip)
        print("Hardware cleanup complete.")


if __name__ == "__main__":
    main()
