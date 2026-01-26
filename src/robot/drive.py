"""High-level robot movement API.

Provides intuitive commands like forward(cm) and turn(degrees).
"""

import time

from robot.config import (
    DEFAULT_STEP_DELAY,
    LEFT_STEPPER_PINS,
    RIGHT_STEPPER_PINS,
    STEPS_PER_CM,
    STEPS_PER_DEGREE,
)
from robot.motors import Stepper


class Robot:
    """High-level robot control with calibrated movement."""

    def __init__(self, chip: int):
        """
        Initialize robot with both motors.

        Args:
            chip: lgpio chip handle from gpiochip_open()
        """
        self.chip = chip
        self.left = Stepper(chip, LEFT_STEPPER_PINS)
        self.right = Stepper(chip, RIGHT_STEPPER_PINS)

    def forward(self, cm: float, delay: float = DEFAULT_STEP_DELAY) -> None:
        """
        Drive forward by specified distance.

        Args:
            cm: Distance to travel in centimeters (negative = backward)
            delay: Step delay in seconds
        """
        steps = int(abs(cm) * STEPS_PER_CM)
        direction = 1 if cm > 0 else -1

        for _ in range(steps):
            self.left.step(direction)
            self.right.step(direction)
            time.sleep(delay)

    def backward(self, cm: float, delay: float = DEFAULT_STEP_DELAY) -> None:
        """Drive backward by specified distance."""
        self.forward(-cm, delay)

    def turn(self, degrees: float, delay: float = DEFAULT_STEP_DELAY) -> None:
        """
        Pivot turn in place.

        Args:
            degrees: Rotation angle (positive = clockwise/right, negative = counter-clockwise/left)
            delay: Step delay in seconds
        """
        steps = int(abs(degrees) * STEPS_PER_DEGREE)
        # Clockwise: left forward, right backward
        left_dir = 1 if degrees > 0 else -1
        right_dir = -1 if degrees > 0 else 1

        for _ in range(steps):
            self.left.step(left_dir)
            self.right.step(right_dir)
            time.sleep(delay)

    def stop(self) -> None:
        """Stop both motors and release holding torque."""
        self.left.stop()
        self.right.stop()
