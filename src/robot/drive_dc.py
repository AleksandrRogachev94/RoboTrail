"""High-level robot movement using DC motors with PID control.

Provides intuitive commands like forward(cm) and turn(degrees).
"""

from time import monotonic, sleep

from robot.config import (
    DC_LEFT_IN1,
    DC_LEFT_IN2,
    DC_LEFT_PWM,
    DC_RIGHT_IN1,
    DC_RIGHT_IN2,
    DC_RIGHT_PWM,
    HEADING_PID_KD,
    HEADING_PID_KI,
    HEADING_PID_KP,
)
from robot.dc_motor_pid import DCMotorPID
from robot.pid import PID
from sensors.encoder import list_encoder_devices
from sensors.imu import IMU

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

        # IMU and heading control
        self.imu = IMU()  # Calibrates gyro on init
        self.heading_pid = PID(kp=HEADING_PID_KP, ki=HEADING_PID_KI, kd=HEADING_PID_KD)
        self._heading = 0.0  # Current heading in degrees (integrated from gyro)
        self._last_heading_time = 0.0

        # Data logging for plots
        self.history = []

    def _update_heading(self, dt: float) -> None:
        """Update heading by integrating gyro Z reading."""
        gyro_z = self.imu.read_gyro_z()  # deg/s
        self._heading += gyro_z * dt

    def forward(self, cm: float, velocity: float = DEFAULT_VELOCITY) -> None:
        """
        Drive forward by specified distance with heading correction.

        Args:
            cm: Distance in centimeters (negative = backward)
            velocity: Speed in ticks/sec
        """
        # Calculate target ticks from cm
        target_ticks = cm * TICKS_PER_CM

        # Reset motors and heading PID
        self.left.reset()
        self.right.reset()
        self.heading_pid.reset()
        target_heading = self._heading  # Maintain current heading

        # Determine velocity sign based on direction
        vel = velocity if cm >= 0 else -velocity

        # Control loop - run until both motors reach target
        t = 0.0
        last_time = monotonic()
        while abs(self.left.position) < abs(target_ticks) and abs(
            self.right.position
        ) < abs(target_ticks):
            # Calculate actual dt
            now = monotonic()
            dt_actual = now - last_time
            last_time = now

            # Update heading from IMU (use actual dt)
            self._update_heading(dt_actual)

            # Heading correction (differential velocity)
            heading_error = target_heading - self._heading
            diff = self.heading_pid.update(heading_error, dt_actual)

            # Apply with heading correction (motor PIDs use their own timing)
            left_vel, left_pwm = self.left.update(vel - diff)
            right_vel, right_pwm = self.right.update(vel + diff)

            # Log data
            self.history.append(
                {
                    "t": t,
                    "left_vel": left_vel,
                    "right_vel": right_vel,
                    "left_pwm": left_pwm,
                    "right_pwm": right_pwm,
                    "heading": self._heading,
                    "heading_error": heading_error,
                    "heading_diff": diff,
                }
            )

            t += dt_actual

            # Sleep remainder of DT for consistent timing
            elapsed = monotonic() - now
            if elapsed < DT:
                sleep(DT - elapsed)

        # Stop motors
        self.stop()

    def turn(self, degrees: float, velocity: float = DEFAULT_VELOCITY) -> None:
        """
        Turn by specified degrees using IMU feedback.

        Args:
            degrees: Rotation angle (positive = counterclockwise)
            velocity: Base speed in ticks/sec
        """
        self.left.reset()
        self.right.reset()
        self.heading_pid.reset()

        target_heading = self._heading + degrees

        # Turn until heading reached (within 1 degree)
        t = 0.0
        last_time = monotonic()
        while abs(target_heading - self._heading) > 1.0:
            # Calculate actual dt
            now = monotonic()
            dt_actual = now - last_time
            last_time = now

            # Update heading from IMU (use actual dt)
            self._update_heading(dt_actual)

            # Heading PID for smooth approach
            heading_error = target_heading - self._heading
            adjustment = self.heading_pid.update(heading_error, dt_actual)

            # Differential drive for turning (feedforward handles friction)
            left_vel, left_pwm = self.left.update(-adjustment)
            right_vel, right_pwm = self.right.update(adjustment)

            # Log data
            self.history.append(
                {
                    "t": t,
                    "left_vel": left_vel,
                    "right_vel": right_vel,
                    "left_pwm": left_pwm,
                    "right_pwm": right_pwm,
                    "heading": self._heading,
                    "heading_error": heading_error,
                    "heading_diff": adjustment,
                }
            )

            t += dt_actual

            # Sleep remainder of DT for consistent timing
            elapsed = monotonic() - now
            if elapsed < DT:
                sleep(DT - elapsed)

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
