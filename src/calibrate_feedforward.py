#!/usr/bin/env python3
"""Calibrate feedforward parameters. Run on ground with space ahead."""

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


def run_and_measure(left, right, left_enc, right_enc, pwm, duration=1.0):
    """Run motors at PWM, poll encoders, return average velocity."""
    left_enc.reset()
    right_enc.reset()
    left.set_speed(pwm)
    right.set_speed(pwm)

    end = time.monotonic() + duration
    while time.monotonic() < end:
        _ = left_enc.position  # Poll to capture events
        _ = right_enc.position
        time.sleep(0.02)

    left.stop()
    right.stop()
    return (abs(left_enc.position) + abs(right_enc.position)) / 2 / duration


def main():
    print("FEEDFORWARD CALIBRATION - Run on ground with 1m space ahead\n")
    input("Press Enter to start...")

    chip = lgpio.gpiochip_open(4)
    left = DCMotor(chip, DC_LEFT_PWM, DC_LEFT_IN1, DC_LEFT_IN2)
    right = DCMotor(chip, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)
    encoders = list_encoder_devices()
    left_enc = Encoder(encoders[1][0], reversed=True)
    right_enc = Encoder(encoders[0][0])

    try:
        # Find dead zone
        print("Finding dead zone...")
        for pwm in range(25, 60, 5):
            vel = run_and_measure(left, right, left_enc, right_enc, pwm, 0.3)
            print(f"  PWM={pwm}% -> {vel:.0f} ticks/sec")
            if vel > 100:
                dead_zone = pwm
                break
        else:
            dead_zone = 40

        # Measure at two points
        print("\nMeasuring velocity...")
        input("Press Enter for 50% test...")
        vel_50 = run_and_measure(left, right, left_enc, right_enc, 50)
        print(f"  50% -> {vel_50:.0f} ticks/sec")

        input("Press Enter for 90% test...")
        vel_90 = run_and_measure(left, right, left_enc, right_enc, 90)
        print(f"  90% -> {vel_90:.0f} ticks/sec")

        # Calculate
        slope = (90 - 50) / (vel_90 - vel_50) if vel_90 > vel_50 else 0.04

        print(f"\n{'=' * 40}")
        print(f"FEEDFORWARD_OFFSET = {dead_zone}")
        print(f"FEEDFORWARD_SLOPE = {slope:.4f}")
        print(f"{'=' * 40}")

    finally:
        left.stop()
        right.stop()
        left_enc.close()
        right_enc.close()
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
