#!/usr/bin/env python3
"""Calibrate feedforward parameters for DC motors.

Measures:
1. Dead zone (minimum PWM to start moving)
2. Velocity at different PWM levels

Output: FEEDFORWARD_OFFSET and FEEDFORWARD_SLOPE for config.py

Run ON THE GROUND with space to move forward ~30cm per test.
"""

import time

import lgpio

from robot.config import (
    DC_LEFT_IN1,
    DC_LEFT_IN2,
    DC_LEFT_PWM,
    DC_RIGHT_IN1,
    DC_RIGHT_IN2,
    DC_RIGHT_PWM,
)
from robot.dc_motor import DCMotor
from sensors.encoder import Encoder, list_encoder_devices

MEASURE_TIME = 1  # seconds per measurement (short to avoid runaway)


def measure_velocity(
    left: DCMotor, right: DCMotor, left_enc: Encoder, right_enc: Encoder, pwm: float
) -> float:
    """Run both motors at PWM, return average velocity."""
    left_enc.reset()
    right_enc.reset()

    # Left motor needs negative speed due to swapped pins in config
    left.set_speed(-pwm)
    right.set_speed(pwm)
    time.sleep(MEASURE_TIME)
    left.stop()
    right.stop()

    left_vel = abs(left_enc.position) / MEASURE_TIME
    right_vel = abs(right_enc.position) / MEASURE_TIME
    return (left_vel + right_vel) / 2


def find_dead_zone(
    left: DCMotor, right: DCMotor, left_enc: Encoder, right_enc: Encoder
) -> float:
    """Find minimum PWM where robot starts moving."""
    print("Finding dead zone (ramping up PWM until wheels move)...")

    for pwm in range(20, 80, 5):
        left_enc.reset()
        right_enc.reset()

        # Left motor needs negative speed due to swapped pins
        left.set_speed(-pwm)
        right.set_speed(pwm)
        time.sleep(0.3)
        left.stop()
        right.stop()

        moved = abs(left_enc.position) + abs(right_enc.position)
        if moved > 20:  # Robot is moving
            print(f"  Robot starts moving at PWM={pwm}%")
            return float(pwm)

        time.sleep(0.2)

    print("  Could not detect motion, using 40%")
    return 40.0


def main():
    print("=" * 50)
    print("FEEDFORWARD CALIBRATION")
    print("=" * 50)
    print("Run ON THE GROUND with ~50cm clear space ahead.")
    print("Robot will move forward in short bursts.\n")
    input("Press Enter when ready...")

    chip = lgpio.gpiochip_open(4)

    # Setup motors
    left_motor = DCMotor(chip, DC_LEFT_PWM, DC_LEFT_IN1, DC_LEFT_IN2)
    right_motor = DCMotor(chip, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)

    # Setup encoders
    encoders = list_encoder_devices()
    right_enc = Encoder(encoders[0][0])
    left_enc = Encoder(encoders[1][0], reversed=True)

    try:
        # Step 1: Find dead zone
        dead_zone = find_dead_zone(left_motor, right_motor, left_enc, right_enc)
        time.sleep(0.5)

        # Step 2: Measure velocity at different PWM levels
        print("\nMeasuring velocity at different PWM levels...")
        print("(Robot will move forward ~15cm each time)\n")
        measurements = []

        for pwm in [50, 70, 90]:
            input(f"Press Enter to test PWM={pwm}%...")
            vel = measure_velocity(left_motor, right_motor, left_enc, right_enc, pwm)
            measurements.append((pwm, vel))
            print(f"  PWM={pwm}% -> Velocity={vel:.0f} ticks/sec\n")

        # Step 3: Calculate slope
        pwm_low, vel_low = measurements[0]
        pwm_high, vel_high = measurements[-1]

        if vel_high > vel_low:
            slope = (pwm_high - pwm_low) / (vel_high - vel_low)
        else:
            slope = 0.04

        # Results
        print("\n" + "=" * 50)
        print("RESULTS - Update these in config.py:")
        print("=" * 50)
        print(f"FEEDFORWARD_OFFSET = {dead_zone:.1f}")
        print(f"FEEDFORWARD_SLOPE = {slope:.4f}")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        left_motor.stop()
        right_motor.stop()
        left_enc.close()
        right_enc.close()
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
