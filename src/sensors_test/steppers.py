#!/usr/bin/env python3
"""Test script for 28BYJ-48 stepper motors via ULN2003."""

import time

import lgpio

# Motor pin definitions (GPIO numbers)
LEFT_MOTOR = [17, 27, 22, 23]  # IN1, IN2, IN3, IN4
RIGHT_MOTOR = [5, 6, 13, 19]  # IN1, IN2, IN3, IN4
# Half-step sequence (smoother, more torque)
HALF_STEP_SEQ = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1],
]


class Stepper:
    def __init__(self, chip, pins):
        self.chip = chip
        self.pins = pins
        self.step_index = 0

        # Set all pins as outputs
        for pin in pins:
            lgpio.gpio_claim_output(chip, pin)

    def step(self, steps, delay=0.001):
        """Move motor by given number of steps. Positive=forward, negative=backward."""
        direction = 1 if steps > 0 else -1

        for _ in range(abs(steps)):
            self.step_index = (self.step_index + direction) % len(HALF_STEP_SEQ)
            seq = HALF_STEP_SEQ[self.step_index]

            for pin, val in zip(self.pins, seq):
                lgpio.gpio_write(self.chip, pin, val)

            time.sleep(delay)

    def stop(self):
        """Turn off all coils (saves power, but loses holding torque)."""
        for pin in self.pins:
            lgpio.gpio_write(self.chip, pin, 0)


def main():
    # Open GPIO chip (Pi 5 uses gpiochip4)
    chip = lgpio.gpiochip_open(4)

    try:
        left = Stepper(chip, LEFT_MOTOR)
        right = Stepper(chip, RIGHT_MOTOR)

        # STEPS_PER_DEGREE = 2693 / 90  # ≈ 29.9 steps per degree
        # print("Pivoting robot 90 degrees clockwise...")
        # steps = int(90 * STEPS_PER_DEGREE)  # 2693 steps
        # for _ in range(steps):
        #     left.step(1, delay=0.001)
        #     right.step(-1, delay=0.001)

        print("Moving both motors forward...")
        for _ in range(
            4096
        ):  # ~quarter turn (28BYJ-48 has ~4096 steps/rev in half-step)
            left.step(1, delay=0.001)
            right.step(1, delay=0.001)

        time.sleep(0.5)

        print("Moving both motors backward...")
        for _ in range(4096):
            left.step(-1, delay=0.001)
            right.step(-1, delay=0.001)

        print("Done!")
        left.stop()
        right.stop()

    finally:
        lgpio.gpiochip_close(chip)


if __name__ == "__main__":
    main()
