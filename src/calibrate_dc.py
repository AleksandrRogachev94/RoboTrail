#!/usr/bin/env python3
"""Calibrate DC motor odometry parameters.

This script helps you calibrate:
1. TICKS_PER_CM - forward movement accuracy
2. TRACK_WIDTH_CM - turning accuracy (using square test, no in-place rotation needed)

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
    current_ticks_per_cm = 62.5  # From config
    correction_factor = 100.0 / actual_cm
    new_ticks_per_cm = current_ticks_per_cm * correction_factor

    print(f"\n{'=' * 60}")
    print(f"Current TICKS_PER_CM: {current_ticks_per_cm}")
    print(f"Commanded: 100cm, Actual: {actual_cm:.1f}cm")
    print(f"Correction factor: {correction_factor:.4f}")
    print(f"\n>>> NEW TICKS_PER_CM = {new_ticks_per_cm:.1f}")
    print("\nUpdate robot/config.py with this value.")
    print(f"{'=' * 60}")

    return new_ticks_per_cm


def calibrate_track_width(robot: RobotDC):
    """Calibrate TRACK_WIDTH_CM using circle test (arc-based, no in-place rotation).

    Procedure:
    1. Robot drives a full 360° circle using robot.arc()
    2. Measure displacement from starting point
    3. Calculate TRACK_WIDTH correction

    Physics:
    - If TRACK_WIDTH is too small: robot understeers → circle too large → ends outside
    - If TRACK_WIDTH is too large: robot oversteers → circle too small → ends inside
    """
    print(f"\n{'=' * 60}")
    print("CALIBRATING TRACK_WIDTH_CM (Circle Test)")
    print(f"{'=' * 60}")
    print("This test drives a full 360° circle (arc, not in-place turn).")
    print("If calibration is perfect, robot returns to exact starting point.")
    print("\nPreparation:")
    print("1. Place robot in center of at least 1.5m x 1.5m clear area")
    print("2. Mark starting position with tape (X marks the spot)")
    print("3. Note starting heading direction")
    input("\nPress Enter when ready...")

    # Reset pose
    robot.x = 0
    robot.y = 0
    robot._heading = 0

    # Drive a circle: radius 40cm, full 360°
    radius_cm = 40.0
    circumference = 2 * 3.14159 * radius_cm  # ≈ 251 cm

    print(
        f"\nDriving 360° circle (radius={radius_cm}cm, arc length={circumference:.0f}cm)..."
    )
    print("This will take ~10 seconds...")

    robot.arc(radius_cm=radius_cm, arc_length_cm=circumference)

    # Get final pose from odometry
    x_final, y_final, heading_final = robot.get_pose()

    print(f"\n{'=' * 60}")
    print("CIRCLE COMPLETED")
    print(f"{'=' * 60}")
    print(
        f"Odometry final: x={x_final:.1f}cm, y={y_final:.1f}cm, θ={heading_final:.1f}°"
    )
    print("\nNow measure the robot's actual position:")
    print("1. How far is the robot center from the starting mark?")

    displacement_cm = float(input("\nEnter displacement in cm (0 if perfect): "))

    if displacement_cm < 2.0:
        print("\n✓ TRACK_WIDTH_CM calibration is excellent! No change needed.")
        return None

    print("\nIn which direction is the robot displaced?")
    print("1. Inside the circle (toward where the circle center was)")
    print("2. Outside the circle (away from where the circle center was)")
    print("3. Not sure / Other")
    direction = input("Enter 1-2: ").strip()

    current_track_width = 17.0  # From config

    # Physics:
    # Circle circumference = arc_length = 2πR
    # If actual circle is larger → R is larger → robot understeered → TRACK_WIDTH too small
    # If actual circle is smaller → R is smaller → robot oversteered → TRACK_WIDTH too large

    # Approximate: displacement ≈ error in radius
    # Circumference error = 2π × displacement
    # Total arc = 2π × R, so fractional error = displacement / R
    # TRACK_WIDTH correction ≈ TRACK_WIDTH × (displacement / R)

    if (
        direction == "1"
    ):  # Inside → circle too small → oversteered → TRACK_WIDTH too large
        correction_cm = -displacement_cm / radius_cm * current_track_width
        print("Inside → Oversteered → TRACK_WIDTH too large")
    elif (
        direction == "2"
    ):  # Outside → circle too large → understeered → TRACK_WIDTH too small
        correction_cm = displacement_cm / radius_cm * current_track_width
        print("Outside → Understeered → TRACK_WIDTH too small")
    else:
        print("Can't calculate correction without direction info.")
        return None

    new_track_width = current_track_width + correction_cm

    print(f"\n{'=' * 60}")
    print(f"Current TRACK_WIDTH_CM: {current_track_width}")
    print(f"Displacement: {displacement_cm:.1f}cm")
    print(f"Estimated correction: {correction_cm:+.1f}cm")
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
    print("2. TRACK_WIDTH_CM (turning via square test)")
    print("\nYou'll need:")
    print("- 2m x 2m clear floor space")
    print("- Tape measure")
    print("- Tape/marker for floor marks")

    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)

    try:
        print("\n\nSelect calibration:")
        print("1. TICKS_PER_CM only (forward)")
        print("2. TRACK_WIDTH_CM only (square test)")
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
