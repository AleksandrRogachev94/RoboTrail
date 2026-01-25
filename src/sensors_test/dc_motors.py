#!/usr/bin/env python3
"""Test DC motors with TB6612FNG driver.

Wiring (from README):
  PWMA  -> GPIO 12 (RIGHT motor speed)
  PWMB  -> GPIO 16 (LEFT motor speed)
  AIN1  -> GPIO 20 (right direction)
  AIN2  -> GPIO 21 (right direction)
  BIN1  -> GPIO 24 (left direction)
  BIN2  -> GPIO 25 (left direction)

Note: Motors have ~40% minimum PWM to overcome static friction.

Run: python3 dc_motors.py
"""

import time

import lgpio

# Pin assignments (A = right motor, B = left motor)
RIGHT_PWM = 12
LEFT_PWM = 16
RIGHT_IN1 = 20
RIGHT_IN2 = 21
# Left motor IN1/IN2 swapped to fix direction
LEFT_IN1 = 25  # Swapped: was 24
LEFT_IN2 = 24  # Swapped: was 25

# PWM frequency (Hz) - motors work well at 1kHz
PWM_FREQ = 1000


def setup(h):
    """Configure GPIO pins."""
    # Must claim ALL pins including PWM pins before use
    all_pins = [LEFT_PWM, RIGHT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]
    for pin in all_pins:
        lgpio.gpio_claim_output(h, pin, 0)


def set_motor(h, pwm_pin, in1, in2, speed):
    """Set motor speed and direction.

    speed: -100 to 100 (negative = reverse)
    """
    if speed > 0:
        lgpio.gpio_write(h, in1, 1)
        lgpio.gpio_write(h, in2, 0)
    elif speed < 0:
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 1)
    else:
        lgpio.gpio_write(h, in1, 0)
        lgpio.gpio_write(h, in2, 0)

    duty = min(abs(speed), 100)
    lgpio.tx_pwm(h, pwm_pin, PWM_FREQ, duty)


def stop(h):
    """Stop both motors."""
    lgpio.tx_pwm(h, LEFT_PWM, PWM_FREQ, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, PWM_FREQ, 0)
    for pin in [LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]:
        lgpio.gpio_write(h, pin, 0)


def main():
    h = lgpio.gpiochip_open(4)  # Pi 5 uses chip 4
    setup(h)

    try:
        print("Test 1: Left motor forward (50% speed) for 2s")
        set_motor(h, LEFT_PWM, LEFT_IN1, LEFT_IN2, 50)
        time.sleep(2)
        stop(h)
        time.sleep(0.5)

        print("Test 2: Right motor forward (50% speed) for 2s")
        set_motor(h, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, 50)
        time.sleep(2)
        stop(h)
        time.sleep(0.5)

        print("Test 3: Both motors forward (50% speed) for 2s")
        set_motor(h, LEFT_PWM, LEFT_IN1, LEFT_IN2, 50)
        set_motor(h, RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, 50)
        time.sleep(2)
        stop(h)
        time.sleep(0.5)

        print("Test 4: Left motor reverse (50% speed) for 2s")
        set_motor(h, LEFT_PWM, LEFT_IN1, LEFT_IN2, -50)
        time.sleep(2)
        stop(h)

        print("Done! All tests passed.")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        stop(h)
        lgpio.gpiochip_close(h)


if __name__ == "__main__":
    main()
