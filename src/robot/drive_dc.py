"""High-level robot movement using DC motors with PID control.

Provides intuitive commands like forward(cm) and turn(degrees).
"""

import math
from time import sleep

from robot.config import (
    DC_LEFT_IN1,
    DC_LEFT_IN2,
    DC_LEFT_PWM,
    DC_RIGHT_IN1,
    DC_RIGHT_IN2,
    DC_RIGHT_PWM,
    TRACK_WIDTH_MM,
)
from robot.dc_motor_pid import DCMotorPID
from sensors.encoder import list_encoder_devices

# Movement constants - CALIBRATE THESE!
TICKS_PER_CM = 62.5  # Calibrate with real measurement
DEFAULT_VELOCITY = 700  # ticks/sec
DT = 0.02  # 50HZ


class RobotDC:
    """High-level robot control with DC motors and PID."""

    def __init__(self, chip: int):
        """
        Initialize robot with both motors.

        Args:
            chip: lgpio chip handle from gpiochip_open()
        """
        self.chip = chip

        # Find encoder devices
        encoders = list_encoder_devices()
        if len(encoders) < 2:
            raise RuntimeError(f"Expected 2 encoders, found {len(encoders)}")

        # Right encoder is first (GPIO 5/6), Left is second (GPIO 17/27)
        right_enc_path = encoders[0][0]
        left_enc_path = encoders[1][0]

        self.left = DCMotorPID(
            self.chip,
            DC_LEFT_PWM,
            DC_LEFT_IN1,
            DC_LEFT_IN2,
            left_enc_path,
            encoder_reversed=True,
        )
        self.right = DCMotorPID(
            self.chip,
            DC_RIGHT_PWM,
            DC_RIGHT_IN1,
            DC_RIGHT_IN2,
            right_enc_path,
            encoder_reversed=False,
        )

    def forward(self, cm: float, velocity: float = DEFAULT_VELOCITY) -> None:
        """
        Drive forward by specified distance.

        Args:
            cm: Distance in centimeters (negative = backward)
            velocity: Speed in ticks/sec
        """
        # Calculate target ticks from cm
        target_ticks = cm * TICKS_PER_CM

        # Reset motors
        self.left.reset()
        self.right.reset()

        # Determine velocity sign based on direction
        vel = velocity if cm >= 0 else -velocity

        # TODO: Control loop - run until both motors reach target
        while abs(self.left.position) < abs(target_ticks) and abs(
            self.right.position
        ) < abs(target_ticks):
            self.left.update(vel)
            self.right.update(vel)
            sleep(DT)

        # Stop motors
        self.stop()

    def turn(self, degrees: float, velocity: float = DEFAULT_VELOCITY):
        """Simple differential turn (no IMU, uses encoders only)."""
        # Rough estimate: track_width * pi * degrees / 360 = arc length per wheel
        arc_per_wheel = (TRACK_WIDTH_MM / 10) * math.pi * abs(degrees) / 360
        target_ticks = arc_per_wheel * TICKS_PER_CM

        self.left.reset()
        self.right.reset()

        left_vel = -velocity if degrees > 0 else velocity
        right_vel = velocity if degrees > 0 else -velocity

        while abs(self.left.position) < target_ticks:
            self.left.update(left_vel)
            self.right.update(right_vel)
            sleep(DT)

        self.stop()

    def stop(self) -> None:
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def close(self) -> None:
        """Release resources."""
        self.stop()
        self.left.close()
        self.right.close()
