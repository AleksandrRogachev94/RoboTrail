#!/usr/bin/env python3
"""Calibrate feedforward parameters for DC motors.

Measures:
1. Dead zone (minimum PWM to start moving)
2. Velocity at different PWM levels

Output: FEEDFORWARD_OFFSET and FEEDFORWARD_SLOPE for config.py
"""

import time

import lgpio

from robot.config import DC_RIGHT_IN1, DC_RIGHT_IN2, DC_RIGHT_PWM
from robot.dc_motor import DCMotor
from sensors.encoder import Encoder, list_encoder_devices

MEASURE_TIME = 1.0  # seconds per measurement


def measure_velocity(motor: DCMotor, encoder: Encoder, pwm: float) -> float:
    """Run motor at PWM for MEASURE_TIME, return average velocity."""
    encoder.reset()
    motor.set_speed(pwm)
    time.sleep(MEASURE_TIME)
    motor.stop()

    velocity = abs(encoder.position) / MEASURE_TIME
    return velocity


def find_dead_zone(motor: DCMotor, encoder: Encoder) -> float:
    """Find minimum PWM where motor starts moving."""
    print("Finding dead zone...")

    for pwm in range(0, 100, 5):
        encoder.reset()
        motor.set_speed(pwm)
        time.sleep(0.5)
        motor.stop()

        if abs(encoder.position) > 10:  # Motor is moving
            print(f"  Motor starts moving at PWM={pwm}%")
            return float(pwm)

    print("  Could not find dead zone!")
    return 40.0


def main():
    print("Calibrating feedforward parameters...")
    print("Make sure robot wheels are OFF the ground!\n")
    input("Press Enter when ready...")

    chip = lgpio.gpiochip_open(4)

    # Use right motor for calibration
    motor = DCMotor(chip, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)
    encoders = list_encoder_devices()
    encoder = Encoder(encoders[0][0])  # Right encoder

    try:
        # Step 1: Find dead zone
        dead_zone = find_dead_zone(motor, encoder)
        time.sleep(0.5)

        # Step 2: Measure velocity at different PWM levels
        print("\nMeasuring velocity at different PWM levels...")
        measurements = []

        for pwm in [50, 70, 90]:
            vel = measure_velocity(motor, encoder, pwm)
            measurements.append((pwm, vel))
            print(f"  PWM={pwm}% -> Velocity={vel:.0f} ticks/sec")
            time.sleep(0.3)

        # Step 3: Calculate slope (linear regression simplified)
        # slope = (pwm_range) / (velocity_range)
        pwm_low, vel_low = measurements[0]
        pwm_high, vel_high = measurements[-1]

        if vel_high > vel_low:
            slope = (pwm_high - pwm_low) / (vel_high - vel_low)
        else:
            slope = 0.04  # fallback

        # Results
        print("\n" + "=" * 50)
        print("RESULTS - Add these to config.py:")
        print("=" * 50)
        print(f"FEEDFORWARD_OFFSET = {dead_zone:.1f}")
        print(f"FEEDFORWARD_SLOPE = {slope:.4f}")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        motor.stop()
        encoder.close()
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
