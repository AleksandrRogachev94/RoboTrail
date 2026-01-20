#!/usr/bin/env python3
"""Test calibration by driving a square.

If the robot returns to its starting position, calibration is good!
"""

import time

import lgpio

from robot.drive import Robot


def drive_square(robot: Robot, side_cm: float = 25.0):
    """
    Drive a square pattern.

    Args:
        robot: Robot instance
        side_cm: Length of each side in cm (default 25cm = 1m total)
    """
    print(f"Driving a {side_cm}cm x {side_cm}cm square...")

    for i in range(4):
        print(f"  Side {i + 1}: forward {side_cm}cm")
        robot.forward(side_cm)
        time.sleep(0.3)

        print(f"  Turn {i + 1}: 90° right")
        robot.turn(90)
        time.sleep(0.3)

    print("Square complete!")


def main():
    chip = lgpio.gpiochip_open(4)

    try:
        robot = Robot(chip)

        print("\n" + "=" * 50)
        print("SQUARE TEST")
        print("=" * 50)
        print("Place robot on floor and mark starting position.")
        print("Robot will drive a 25cm x 25cm square.")
        print("If it returns to start, calibration is good!")
        input("\nPress Enter to start...")

        drive_square(robot, side_cm=25.0)

        print("\nCheck: Is the robot back at the starting position?")
        print("  - Off by a lot? Re-run calibrate_motors.py")
        print("  - Close but not exact? Fine-tune config.py values")

        robot.stop()

    finally:
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
