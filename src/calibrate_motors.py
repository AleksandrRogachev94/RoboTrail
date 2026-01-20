#!/usr/bin/env python3
"""Motor calibration script.

Run this to determine STEPS_PER_CM and STEPS_PER_DEGREE for your robot.
Update the values in robot/config.py after calibration.
"""

import lgpio

from robot.config import LEFT_MOTOR_PINS, RIGHT_MOTOR_PINS
from robot.motors import Stepper


def calibrate_forward(chip, test_steps: int = 4096):
    """
    Calibrate forward movement.

    Procedure:
    1. Mark starting position
    2. Robot will move forward by test_steps
    3. Measure actual distance traveled (cm)
    4. Calculate STEPS_PER_CM = test_steps / distance_cm
    """
    left = Stepper(chip, LEFT_MOTOR_PINS)
    right = Stepper(chip, RIGHT_MOTOR_PINS)

    print(f"\n{'=' * 50}")
    print("FORWARD CALIBRATION")
    print(f"{'=' * 50}")
    print("1. Place robot on flat surface")
    print("2. Mark starting position (tape on floor)")
    print(f"3. Robot will move forward {test_steps} steps")
    input("\nPress Enter when ready...")

    print("Moving forward...")
    for _ in range(test_steps):
        left.step(1, delay=0.001)
        right.step(1, delay=0.001)

    left.stop()
    right.stop()

    print("\nMotor stopped.")
    distance = float(input("Measure distance traveled (cm): "))

    steps_per_cm = test_steps / distance
    print(f"\n>>> STEPS_PER_CM = {steps_per_cm:.1f}")
    print("    Update this value in robot/config.py")

    return steps_per_cm


def calibrate_rotation(chip, test_steps: int = 2700):
    """
    Calibrate pivot turn.

    Procedure:
    1. Mark robot heading (point a laser or align with a line)
    2. Robot will pivot by test_steps
    3. Measure actual rotation (degrees)
    4. Calculate STEPS_PER_DEGREE = test_steps / degrees
    """
    left = Stepper(chip, LEFT_MOTOR_PINS)
    right = Stepper(chip, RIGHT_MOTOR_PINS)

    print(f"\n{'=' * 50}")
    print("ROTATION CALIBRATION")
    print(f"{'=' * 50}")
    print("1. Place robot on flat surface")
    print("2. Mark initial heading (use laser or align with grid line)")
    print(f"3. Robot will pivot {test_steps} steps (clockwise)")
    input("\nPress Enter when ready...")

    print("Pivoting clockwise...")
    for _ in range(test_steps):
        left.step(1, delay=0.001)  # Left forward
        right.step(-1, delay=0.001)  # Right backward

    left.stop()
    right.stop()

    print("\nMotor stopped.")
    degrees = float(input("Measure rotation angle (degrees): "))

    steps_per_degree = test_steps / degrees
    print(f"\n>>> STEPS_PER_DEGREE = {steps_per_degree:.1f}")
    print("    Update this value in robot/config.py")

    return steps_per_degree


def main():
    chip = lgpio.gpiochip_open(4)

    try:
        print("\nMotor Calibration")
        print("=================")
        print("1. Calibrate forward movement (STEPS_PER_CM)")
        print("2. Calibrate rotation (STEPS_PER_DEGREE)")
        print("3. Both")
        print("q. Quit")

        choice = input("\nChoice: ").strip().lower()

        if choice == "1":
            calibrate_forward(chip)
        elif choice == "2":
            calibrate_rotation(chip)
        elif choice == "3":
            spc = calibrate_forward(chip)
            spd = calibrate_rotation(chip)
            print(f"\n{'=' * 50}")
            print("SUMMARY - Update robot/config.py with:")
            print(f"{'=' * 50}")
            print(f"STEPS_PER_CM = {spc:.1f}")
            print(f"STEPS_PER_DEGREE = {spd:.1f}")
        elif choice == "q":
            print("Exiting.")
        else:
            print("Invalid choice.")

    finally:
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
