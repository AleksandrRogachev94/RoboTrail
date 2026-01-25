#!/usr/bin/env python3
"""Motor calibration script using gyro for rotation measurement.

Run this to determine STEPS_PER_CM and STEPS_PER_DEGREE for your robot.
Update the values in robot/config.py after calibration.
"""

import time

import lgpio

from robot.config import LEFT_MOTOR_PINS, RIGHT_MOTOR_PINS
from robot.motors import Stepper
from sensors.imu import IMU


def calibrate_forward(chip, test_steps: int = 4096):
    """
    Calibrate forward movement.

    Procedure:
    1. Mark starting position
    2. Robot moves forward by test_steps
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


def calibrate_rotation(chip, target_degrees: float = 360.0):
    """
    Calibrate pivot turn using gyro measurement.

    Procedure:
    1. Gyro calibrates automatically (keep robot still)
    2. Robot pivots while integrating gyro readings (using accurate timing)
    3. Gyro reports actual rotation
    4. Calculate STEPS_PER_DEGREE
    """
    left = Stepper(chip, LEFT_MOTOR_PINS)
    right = Stepper(chip, RIGHT_MOTOR_PINS)

    print(f"\n{'=' * 50}")
    print("ROTATION CALIBRATION (with gyro)")
    print(f"{'=' * 50}")
    print("1. Place robot on flat surface")
    print("2. Keep robot COMPLETELY STILL for gyro calibration")
    input("\nPress Enter when ready...")

    # Initialize and calibrate gyro
    imu = IMU(calibrate=True)

    # Use theoretical estimate as starting point
    # With 40mm wheel, 130mm track: ~37 steps/degree
    estimated_steps_per_degree = 37.0
    test_steps = int(target_degrees * estimated_steps_per_degree)

    print(f"\nRobot will pivot ~{target_degrees}° ({test_steps} steps)")
    input("Press Enter to start rotation...")

    # Integrate gyro during rotation with accurate timing
    total_rotation = 0.0
    step_delay = 0.001
    last_time = time.perf_counter()

    print("Pivoting clockwise...")
    for i in range(test_steps):
        # Measure actual elapsed time
        now = time.perf_counter()
        dt = now - last_time
        last_time = now

        # Read and integrate gyro
        gz = imu.read_gyro_z()  # deg/s
        total_rotation += gz * dt

        # Execute step
        left.step(1, delay=step_delay)
        right.step(-1, delay=step_delay)

    left.stop()
    right.stop()

    # Take absolute value (direction doesn't matter for calibration)
    measured_degrees = abs(total_rotation)

    print("\n--- Results ---")
    print(f"Steps executed: {test_steps}")
    print(f"Gyro measured:  {measured_degrees:.1f}°")

    if measured_degrees > 1.0:  # Avoid division by near-zero
        steps_per_degree = test_steps / measured_degrees
        print(f"\n>>> STEPS_PER_DEGREE = {steps_per_degree:.1f}")
        print("    Update this value in robot/config.py")
        return steps_per_degree
    else:
        print("\nERROR: Gyro measured almost no rotation.")
        print("Check: Is gyro working? Is robot actually moving?")
        return None


def main():
    chip = lgpio.gpiochip_open(4)

    try:
        print("\nMotor Calibration")
        print("=================")
        print("1. Calibrate forward movement (STEPS_PER_CM)")
        print("2. Calibrate rotation (STEPS_PER_DEGREE) - uses gyro")
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
            if spd:
                print(f"STEPS_PER_DEGREE = {spd:.1f}")
        elif choice == "q":
            print("Exiting.")
        else:
            print("Invalid choice.")

    finally:
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
