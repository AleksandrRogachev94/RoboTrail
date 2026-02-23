#!/usr/bin/env python3
"""Calibrate DC motor odometry parameters.

This script helps you calibrate:
1. TICKS_PER_CM - forward movement accuracy
2. TRACK_WIDTH_CM - turning accuracy (using in-place rotation)

Run on robot with at least 2m x 2m of clear floor space.
"""

import lgpio

from robot.drive_dc import RobotDC


def calibrate_ticks_per_cm(robot: RobotDC):
    """Calibrate TICKS_PER_CM for forward movement.

    Procedure:
    1. Place tape/mark at starting position
    2. Robot drives forward 100cm (commanded)
    3. Measure actual distance traveled
    4. Calculate correction factor
    """
    print(f"\n{'=' * 60}")
    print("CALIBRATING TICKS_PER_CM")
    print(f"{'=' * 60}")
    print("1. Place robot at a marked starting position")
    print("2. Ensure at least 120cm of clear space ahead")
    print("3. Mark the starting position (tape on floor)")
    input("\nPress Enter when ready...")

    print("\nDriving forward 100cm...")
    robot.forward(100)

    x, y, heading = robot.get_pose()
    print(f"\nOdometry thinks it moved: {x:.1f}cm")
    print("\nMeasure the ACTUAL distance traveled with a tape measure.")
    print("Measure from the starting mark to the center of the robot now.")

    actual_cm = float(input("\nEnter actual distance in cm: "))

    # Calculate correction
    from robot.config import TICKS_PER_CM

    correction_factor = 100.0 / actual_cm
    new_ticks_per_cm = TICKS_PER_CM * correction_factor

    print(f"\n{'=' * 60}")
    print(f"Current TICKS_PER_CM: {TICKS_PER_CM}")
    print(f"Commanded: 100cm, Actual: {actual_cm:.1f}cm")
    print(f"Correction factor: {correction_factor:.4f}")
    print(f"\n>>> NEW TICKS_PER_CM = {new_ticks_per_cm:.1f}")
    print("\nUpdate robot/config.py with this value.")
    print(f"{'=' * 60}")

    return new_ticks_per_cm


def calibrate_track_width(robot: RobotDC):
    """Calibrate TRACK_WIDTH_CM using in-place rotation.

    Procedure:
    1. Mark starting heading direction on floor
    2. Robot turns 360° using turn() (in-place rotation)
    3. Compare commanded vs actual rotation
    4. Adjust TRACK_WIDTH_CM based on discrepancy

    Physics:
    - Arc length per wheel = (TRACK_WIDTH/2) × θ
    - If TRACK_WIDTH is too small: wheels don't move far enough → under-rotation
    - If TRACK_WIDTH is too large: wheels move too far → over-rotation
    """
    print(f"\n{'=' * 60}")
    print("CALIBRATING TRACK_WIDTH_CM (In-Place Rotation)")
    print(f"{'=' * 60}")
    print("This test rotates 360° in place.")
    print("If calibration is perfect, robot returns to exact starting heading.")
    print("\nPreparation:")
    print("1. Place robot on flat surface")
    print("2. Mark a line on the floor aligned with the robot's heading")
    print("3. Mark a reference point on the robot chassis")
    input("\nPress Enter when ready...")

    # Reset pose
    robot.reset_pose()

    print("\nTurning 360° in place...")
    robot.turn(360)

    _, _, heading_final = robot.get_pose()

    print(f"\n{'=' * 60}")
    print("ROTATION COMPLETED")
    print(f"{'=' * 60}")
    print(f"IMU final heading: {heading_final:.1f}°")
    print("\nNow check the robot's actual heading:")
    print("Is the reference mark aligned with the floor line?")
    print("\nOptions:")
    print("1. Robot under-rotated (didn't complete full 360°)")
    print("2. Robot is perfectly aligned")
    print("3. Robot over-rotated (went past 360°)")

    choice = input("\nEnter 1-3: ").strip()

    if choice == "2":
        print("\n✓ TRACK_WIDTH_CM calibration is excellent! No change needed.")
        return None

    from robot.config import TRACK_WIDTH_CM

    actual_degrees = float(
        input("\nEstimate actual rotation in degrees (e.g. 350 or 370): ")
    )

    # If robot turned X° when asked for 360°:
    # correction_factor = 360 / actual
    # new_track_width = current × correction_factor
    new_track_width = TRACK_WIDTH_CM * (360.0 / actual_degrees)

    print(f"\n{'=' * 60}")
    print(f"Current TRACK_WIDTH_CM: {TRACK_WIDTH_CM}")
    print(f"Commanded: 360°, Actual: {actual_degrees:.1f}°")
    print(f"\n>>> NEW TRACK_WIDTH_CM = {new_track_width:.1f}")
    print("\nUpdate robot/config.py, then re-run this test to verify.")
    print(f"{'=' * 60}")

    return new_track_width


def main():
    print("\n" + "=" * 60)
    print("DC MOTOR ODOMETRY CALIBRATION")
    print("=" * 60)
    print("\nThis will calibrate:")
    print("1. TICKS_PER_CM (forward movement)")
    print("2. TRACK_WIDTH_CM (turning via in-place rotation)")
    print("\nYou'll need:")
    print("- 2m x 2m clear floor space")
    print("- Tape measure")
    print("- Tape/marker for floor marks")

    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)

    try:
        print("\n\nSelect calibration:")
        print("1. TICKS_PER_CM only (forward)")
        print("2. TRACK_WIDTH_CM only (rotation)")
        print("3. Both (recommended)")
        choice = input("\nChoice (1-3): ").strip()

        if choice in ["1", "3"]:
            calibrate_ticks_per_cm(robot)
            input(
                "\nPress Enter to continue to TRACK_WIDTH test (or Ctrl+C to quit)..."
            )

        if choice in ["2", "3"]:
            calibrate_track_width(robot)

        print("\n✓ Calibration complete!")
        print("Remember to update robot/config.py with the new values.")

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    finally:
        robot.close()
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
