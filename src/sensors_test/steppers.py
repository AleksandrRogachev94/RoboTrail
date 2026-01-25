#!/usr/bin/env python3
"""Test script for 28BYJ-48 stepper motors via ULN2003."""

import time

import lgpio

from robot.config import LEFT_MOTOR_PINS, RIGHT_MOTOR_PINS
from robot.motors import Stepper


def main():
    # Open GPIO chip (Pi 5 uses gpiochip4)
    chip = lgpio.gpiochip_open(4)

    try:
        left = Stepper(chip, LEFT_MOTOR_PINS)
        right = Stepper(chip, RIGHT_MOTOR_PINS)

        print("Moving both motors forward...")
        for _ in range(
            4096
        ):  # ~quarter turn (28BYJ-48 has ~4096 steps/rev in half-step)
            left.step(1)
            right.step(1)
            time.sleep(0.001)

        time.sleep(0.5)

        print("Moving both motors backward...")
        for _ in range(4096):
            left.step(-1)
            right.step(-1)
            time.sleep(0.001)

        print("Done!")
        left.stop()
        right.stop()

    finally:
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
