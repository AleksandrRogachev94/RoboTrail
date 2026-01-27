"""DC motor with integrated encoder and PID velocity control."""

import time

from robot.config import PID_KD, PID_KI, PID_KP
from robot.dc_motor import DCMotor
from robot.pid import PID
from sensors.encoder import Encoder


class DCMotorPID:
    """Single DC motor with closed-loop velocity control.

    Combines:
    - DCMotor: PWM control
    - Encoder: Position/velocity feedback
    - PID: Velocity controller
    """

    def __init__(
        self,
        chip: int,
        pwm_pin: int,
        in1_pin: int,
        in2_pin: int,
        encoder_path: str,
        encoder_reversed: bool = False,
    ):
        """
        Initialize motor with PID control.

        Args:
            chip: lgpio chip handle
            pwm_pin: GPIO pin for PWM
            in1_pin: Direction control pin 1
            in2_pin: Direction control pin 2
            encoder_path: Path to encoder device (e.g., '/dev/input/event0')
            encoder_reversed: If True, negate encoder readings
        """
        self.motor = DCMotor(chip, pwm_pin, in1_pin, in2_pin)
        self.encoder = Encoder(encoder_path, reversed=encoder_reversed)
        self.pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)

        # State for velocity calculation
        self._last_pos = 0
        self._last_time = time.monotonic()
        self._velocity = 0.0

    def update(self, target_velocity: float) -> float:
        """
        Run one PID iteration. Call this at ~50Hz.

        Args:
            target_velocity: Desired velocity in ticks/sec

        Returns:
            Current measured velocity
        """
        now = time.monotonic()
        dt = now - self._last_time

        # Read encoder position
        pos = self.encoder.position

        # Calculate velocity = (pos - last_pos) / dt
        velocity = (pos - self._last_pos) / dt

        # Calculate error and get PID output
        error = target_velocity - velocity
        output = self.pid.update(error, dt)

        # Apply output to motor
        self.motor.set_speed(output)

        # Update state variables
        self._last_pos = pos
        self._last_time = now
        self._velocity = velocity

        return velocity

    @property
    def position(self) -> int:
        """Total ticks traveled (cumulative)."""
        return self.encoder.position

    def reset(self) -> None:
        """Reset encoder position and PID state."""
        self.encoder.reset()
        self.pid.reset()
        self._last_pos = 0
        self._last_time = time.monotonic()
        self._velocity = 0.0

    def stop(self) -> None:
        """Stop motor and reset PID."""
        self.motor.stop()
        self.pid.reset()

    def close(self) -> None:
        """Release resources."""
        self.stop()
        self.encoder.close()
