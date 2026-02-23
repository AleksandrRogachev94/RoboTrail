#!/usr/bin/env python3
"""Calibrate DC motor odometry parameters.

This script helps you calibrate:
  TICKS_PER_CM - forward movement accuracy

Run on robot with at least 120cm of clear floor space ahead.
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


def main():
    print("\n" + "=" * 60)
    print("DC MOTOR ODOMETRY CALIBRATION")
    print("=" * 60)
    print("\nThis will calibrate TICKS_PER_CM (forward distance accuracy).")
    print("\nYou'll need:")
    print("- 120cm of clear floor space ahead")
    print("- Tape measure")
    print("- Tape/marker for floor marks")

    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)

    try:
        calibrate_ticks_per_cm(robot)
        print("\n✓ Calibration complete!")
        print("Remember to update robot/config.py with the new value.")

    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    finally:
        robot.close()
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
