"""High-level robot movement using DC motors with PID control.

Provides intuitive commands like forward(cm) and turn(degrees).
"""

import math
from time import monotonic, sleep

from robot.config import (
    DC_LEFT_IN1,
    DC_LEFT_IN2,
    DC_LEFT_PWM,
    DC_RIGHT_IN1,
    DC_RIGHT_IN2,
    DC_RIGHT_PWM,
    DEFAULT_VELOCITY,
    DT,
    HEADING_PID_KD,
    HEADING_PID_KI,
    HEADING_PID_KP,
    TICKS_PER_CM,
    TRACK_WIDTH_CM,
)
from robot.dc_motor_pid import DCMotorPID
from robot.pid import PID
from sensors.encoder import list_encoder_devices
from sensors.imu import IMU


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
        self.reset_pose()

        # Data logging for plots
        self.history = []

    def _update_heading(self, dt: float) -> float:
        """Update heading from IMU gyro.

        Returns:
            Angular velocity in rad/s (for position update).
        """
        gyro_z = self.imu.read_gyro_z()  # deg/s
        self._heading += gyro_z * dt
        return math.radians(gyro_z)  # rad/s

    def _update_position(
        self, left_vel: float, right_vel: float, omega: float, dt: float
    ) -> None:
        """Update x, y position using RK2 integration.

        Args:
            left_vel: Left wheel velocity in ticks/sec
            right_vel: Right wheel velocity in ticks/sec
            omega: Angular velocity in rad/s (from _update_heading)
            dt: Time step in seconds
        """
        # Linear velocity from encoders
        v = (left_vel + right_vel) / 2 / TICKS_PER_CM  # cm/s

        # RK2: use midpoint heading for curved path
        theta = math.radians(self._heading)
        theta_mid = theta - (omega * dt) / 2  # Go back to midpoint

        # Update position
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt

    def _max_arc_velocity(self, radius_cm: float) -> float:
        """Compute max center velocity to prevent outer wheel saturation.

        Args:
            radius_cm: Turning radius (positive or negative).

        Returns:
            Max safe velocity in ticks/sec.
        """
        # Outer wheel travels (R + W/2) / R times the center distance
        outer_factor = (abs(radius_cm) + TRACK_WIDTH_CM / 2) / abs(radius_cm)
        return DEFAULT_VELOCITY / outer_factor

    def arc_to_point(
        self, target_x: float, target_y: float
    ) -> tuple[float, float] | tuple[None, float]:
        """Compute arc parameters to reach a target point from current pose.

        Finds the unique circular arc that:
        1. Starts at current position (self.x, self.y)
        2. Has initial direction matching current heading (self._heading)
        3. Passes through (target_x, target_y)

        Args:
            target_x: Target X position in cm (world frame).
            target_y: Target Y position in cm (world frame).

        Returns:
            (radius_cm, arc_length_cm) for use with arc(), or
            (None, distance_cm) if target is straight ahead (use forward()).
        """
        # Vector to target in world frame
        dx = target_x - self.x
        dy = target_y - self.y

        # Transform to robot frame (ROS: x=forward, y=left, heading=0 → +x)
        theta = math.radians(self._heading)
        # local_x = forward, local_y = left
        local_x = dx * math.cos(theta) + dy * math.sin(theta)
        local_y = -dx * math.sin(theta) + dy * math.cos(theta)

        # Distance to target
        L = math.hypot(local_x, local_y)

        # If target is nearly straight ahead, use forward()
        if abs(local_y) < 0.5:  # Less than 5mm lateral offset
            return (None, local_x)  # None signals "use forward()"

        # Arc geometry: R = L² / (2 * local_y)
        # Positive local_y = target is to the left = positive radius (left turn)
        radius = (L**2) / (2 * local_y)

        # Arc angle (radians) - how much we turn
        arc_angle = 2 * math.atan2(local_y, local_x)

        # Arc length
        arc_length = abs(radius * arc_angle)

        return (radius, arc_length)

    def move_to(self, target_x: float, target_y: float) -> None:
        """Move to a target point using arc or straight-line motion.

        Computes the appropriate arc (or straight line) from current pose
        to target and executes it.

        Args:
            target_x: Target X position in cm (world frame).
            target_y: Target Y position in cm (world frame).
        """
        result = self.arc_to_point(target_x, target_y)

        if result[0] is None:
            # Target is straight ahead
            self.forward(result[1])
        else:
            # Use arc motion
            radius, arc_length = result
            self.arc(radius, arc_length)

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
            omega = self._update_heading(dt_actual)

            # Heading correction (differential velocity)
            heading_error = target_heading - self._heading
            diff = self.heading_pid.update(heading_error, dt_actual)

            # Apply with heading correction (motor PIDs use their own timing)
            left_vel, left_pwm = self.left.update(vel - diff)
            right_vel, right_pwm = self.right.update(vel + diff)

            # Update position with current velocities
            self._update_position(left_vel, right_vel, omega, dt_actual)

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

    def arc(
        self,
        radius_cm: float,
        arc_length_cm: float,
        velocity: float | None = None,
    ) -> None:
        """
        Drive in a circular arc.

        Args:
            radius_cm: Turning radius from robot center.
                       Positive = turn left (CCW), Negative = turn right (CW).
            arc_length_cm: Distance to travel along the arc (positive = forward).
            velocity: Target linear velocity at robot center (ticks/sec).
                      If None, auto-computes max safe velocity for the radius.
        """
        # Auto-compute or clamp velocity to prevent outer wheel saturation
        max_vel = self._max_arc_velocity(radius_cm)
        if velocity is None:
            velocity = max_vel
        else:
            velocity = min(velocity, max_vel)
        print(
            f"Arc: radius={radius_cm}cm, velocity={velocity:.0f} ticks/s (max={max_vel:.0f})"
        )

        # Reset motors and heading PID
        self.left.reset()
        self.right.reset()
        self.heading_pid.reset()

        # Angular velocity around arc center (rad/s in cm units)
        omega = velocity / TICKS_PER_CM / abs(radius_cm)

        # Compute wheel velocities based on turn direction
        if radius_cm > 0:  # Left turn: left wheel is inner
            left_vel = omega * (radius_cm - TRACK_WIDTH_CM / 2) * TICKS_PER_CM
            right_vel = omega * (radius_cm + TRACK_WIDTH_CM / 2) * TICKS_PER_CM
            turn_sign = 1  # Heading increases (CCW)
        else:  # Right turn: right wheel is inner
            left_vel = omega * (abs(radius_cm) + TRACK_WIDTH_CM / 2) * TICKS_PER_CM
            right_vel = omega * (abs(radius_cm) - TRACK_WIDTH_CM / 2) * TICKS_PER_CM
            turn_sign = -1  # Heading decreases (CW)

        # Target arc in ticks (use average of both wheels' paths)
        target_ticks = arc_length_cm * TICKS_PER_CM
        start_heading = self._heading

        # Control loop
        t = 0.0
        last_time = monotonic()
        while True:
            # Check progress using average of both encoders
            avg_ticks = (abs(self.left.position) + abs(self.right.position)) / 2
            if avg_ticks >= target_ticks:
                break

            # Calculate actual dt
            now = monotonic()
            dt_actual = now - last_time
            last_time = now

            # Update heading from IMU
            omega_actual = self._update_heading(dt_actual)

            # Expected heading based on arc progress
            progress_angle = avg_ticks / TICKS_PER_CM / abs(radius_cm)  # radians
            expected_heading = start_heading + math.degrees(progress_angle) * turn_sign

            # Heading correction (PID tracks expected curved path)
            heading_error = expected_heading - self._heading
            diff = self.heading_pid.update(heading_error, dt_actual)

            # Apply velocities with heading correction
            actual_left_vel, left_pwm = self.left.update(left_vel - diff)
            actual_right_vel, right_pwm = self.right.update(right_vel + diff)

            # Update position
            self._update_position(
                actual_left_vel, actual_right_vel, omega_actual, dt_actual
            )

            # Log data
            self.history.append(
                {
                    "t": t,
                    "left_vel": actual_left_vel,
                    "right_vel": actual_right_vel,
                    "left_pwm": left_pwm,
                    "right_pwm": right_pwm,
                    "heading": self._heading,
                    "expected_heading": expected_heading,
                    "heading_error": heading_error,
                    "heading_diff": diff,
                }
            )

            t += dt_actual

            # Sleep remainder of DT
            elapsed = monotonic() - now
            if elapsed < DT:
                sleep(DT - elapsed)

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
            omega = self._update_heading(dt_actual)

            # Heading PID for smooth approach
            heading_error = target_heading - self._heading
            adjustment = self.heading_pid.update(heading_error, dt_actual)

            # Differential drive for turning (feedforward handles friction)
            left_vel, left_pwm = self.left.update(-adjustment)
            right_vel, right_pwm = self.right.update(adjustment)

            # Update position with current velocities
            self._update_position(left_vel, right_vel, omega, dt_actual)

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

    def get_pose(self) -> tuple[float, float, float]:
        """Return current pose (x, y, heading) in cm/degrees."""
        return (self.x, self.y, self._heading)

    def reset_pose(self) -> None:
        """Reset pose to origin."""
        self._heading = 0.0  # Current heading in degrees
        self.x = 0.0  # Current x position in cm
        self.y = 0.0  # Current y position in cm

    def stop(self) -> None:
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def close(self) -> None:
        """Release resources."""
        self.stop()
        self.left.close()
        self.right.close()
