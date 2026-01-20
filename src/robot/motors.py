"""Low-level stepper motor control for 28BYJ-48 via ULN2003.

This module only understands steps - no knowledge of cm or degrees.
Use drive.py for high-level movement commands.
"""

import lgpio

# Half-step sequence (8 steps per cycle, smoother movement, more torque)
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
    """Controls a single 28BYJ-48 stepper motor via ULN2003 driver."""

    def __init__(self, chip: int, pins: list[int]):
        """
        Initialize stepper motor.

        Args:
            chip: lgpio chip handle from gpiochip_open()
            pins: List of 4 GPIO pins [IN1, IN2, IN3, IN4]
        """
        self.chip = chip
        self.pins = pins
        self.step_index = 0

        # Configure pins as outputs
        for pin in pins:
            lgpio.gpio_claim_output(chip, pin)

    def step(self, direction: int = 1) -> None:
        """
        Advance motor by one step.

        Args:
            direction: 1 for forward, -1 for backward
        """
        self.step_index = (self.step_index + direction) % len(HALF_STEP_SEQ)
        seq = HALF_STEP_SEQ[self.step_index]

        for pin, val in zip(self.pins, seq):
            lgpio.gpio_write(self.chip, pin, val)

    def stop(self) -> None:
        """Turn off all coils (saves power, loses holding torque)."""
        for pin in self.pins:
            lgpio.gpio_write(self.chip, pin, 0)
