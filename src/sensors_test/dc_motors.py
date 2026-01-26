#!/usr/bin/env python3
"""Test DC motors with TB6612FNG driver.

Note: Motors have ~40% minimum PWM to overcome static friction.

Run: python3 -m sensors_test.dc_motors
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


def main():
    h = lgpio.gpiochip_open(4)  # Pi 5 uses chip 4

    left = DCMotor(h, DC_LEFT_PWM, DC_LEFT_IN1, DC_LEFT_IN2)
    right = DCMotor(h, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)

    try:
        print("Test 1: Left motor forward (50%) for 2s")
        left.set_speed(50)
        time.sleep(2)
        left.stop()
        time.sleep(0.5)

        print("Test 2: Right motor forward (50%) for 2s")
        right.set_speed(50)
        time.sleep(2)
        right.stop()
        time.sleep(0.5)

        print("Test 3: Both motors forward (50%) for 2s")
        left.set_speed(50)
        right.set_speed(50)
        time.sleep(2)
        left.stop()
        right.stop()
        time.sleep(0.5)

        print("Test 4: Left motor reverse (50%) for 2s")
        left.set_speed(-50)
        time.sleep(2)
        left.stop()

        print("Done!")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        left.stop()
        right.stop()
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
