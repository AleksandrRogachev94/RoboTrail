#!/usr/bin/env python3
"""Test script for DC motor drive system.

Use this to:
1. Test forward/backward movement
2. Test turning
3. Calibrate TICKS_PER_CM

Run on Pi: python3 drive_dc_test.py
"""

import lgpio

from robot.drive_dc import RobotDC


def main():
    print("Initializing robot...")
    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)

    try:
        # Test 1: Forward 30cm
        print("\n=== Test: Forward 30cm ===")
        print("Place robot at a marked starting point.")
        input("Press Enter to start...")

        robot.forward(30)

        print("Done! Measure actual distance traveled.")
        actual = input("Enter actual distance in cm (or press Enter to skip): ")
        if actual:
            actual = float(actual)
            correction = 30 / actual
            print(f"Correction factor: {correction:.3f}")
            print(f"New TICKS_PER_CM = {50 * correction:.1f}")

        # Test 2: Backward 30cm
        print("\n=== Test: Backward 30cm ===")
        input("Press Enter to go back...")
        robot.forward(-30)
        print("Done!")

        # Test 3: Turn 90 degrees
        print("\n=== Test: Turn 90 degrees CCW ===")
        input("Press Enter to turn...")
        robot.turn(90)
        print("Done! Check if it turned ~90 degrees.")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.close()
        lgpio.gpiochip_close(chip)
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
