"""High-level robot movement using DC motors with PID control.

Provides intuitive commands: forward(cm), turn(degrees), move_to(x, y),
and follow_path(waypoints).

Motion model: turn-in-place to face target, then drive straight.

Uses trapezoidal velocity profiles for smooth acceleration/deceleration.
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
    DT,
    HEADING_PID_KD,
    HEADING_PID_KI,
    HEADING_PID_KP,
    MAX_FORWARD_VELOCITY,
    MAX_TURN_VELOCITY,
    MIN_SPEED_FACTOR,
    MIN_TURN_FACTOR,
    RAMP_ANGLE_DEG,
    RAMP_DISTANCE_CM,
    TICKS_PER_CM,
    TURN_PID_FINE_GAIN,
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

    # ── Internals ──────────────────────────────────────────────────────

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

    def _log(self, t, left_vel, right_vel, left_pwm, right_pwm, heading_error, diff):
        """Append a data point to history."""
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

    @staticmethod
    def _speed_factor(
        progress: float, total: float, ramp: float, min_factor: float
    ) -> float:
        """Compute trapezoidal speed factor based on progress.

        Args:
            progress: How far along (cm or degrees traveled).
            total: Total distance/angle to cover.
            ramp: Ramp distance/angle for acceleration and deceleration.
            min_factor: Minimum speed factor (0..1).

        Returns:
            Speed multiplier between min_factor and 1.0.
        """
        if total <= 2 * ramp:
            # Short move: triangle profile (no cruise phase)
            half = total / 2
            if progress < half:
                frac = progress / half if half > 0 else 1.0
            else:
                frac = (total - progress) / half if half > 0 else 1.0
            return min_factor + (1.0 - min_factor) * frac

        # Normal trapezoidal: ramp up → cruise → ramp down
        remaining = total - progress
        if progress < ramp:
            frac = progress / ramp
            return min_factor + (1.0 - min_factor) * frac
        elif remaining < ramp:
            frac = remaining / ramp
            return min_factor + (1.0 - min_factor) * frac
        else:
            return 1.0

    # ── Movement Commands ──────────────────────────────────────────────

    def forward(self, cm: float, velocity: float = MAX_FORWARD_VELOCITY) -> None:
        """Drive forward with trapezoidal velocity profile and heading hold.

        Ramps up over the first RAMP_DISTANCE_CM, cruises, then ramps down
        over the last RAMP_DISTANCE_CM. Heading PID maintains straight line.

        Args:
            cm: Distance in centimeters (negative = backward).
            velocity: Peak speed in ticks/sec.
        """
        target_ticks = abs(cm) * TICKS_PER_CM
        total_cm = abs(cm)
        direction = 1.0 if cm >= 0 else -1.0

        self.left.reset()
        self.right.reset()
        self.heading_pid.reset()
        target_heading = self._heading

        t = 0.0
        last_time = monotonic()
        while True:
            avg_ticks = (abs(self.left.position) + abs(self.right.position)) / 2
            if avg_ticks >= target_ticks:
                break

            now = monotonic()
            dt_actual = now - last_time
            last_time = now

            omega = self._update_heading(dt_actual)

            # Velocity profile
            progress_cm = avg_ticks / TICKS_PER_CM
            factor = self._speed_factor(
                progress_cm, total_cm, RAMP_DISTANCE_CM, MIN_SPEED_FACTOR
            )
            vel = direction * velocity * factor

            # Heading correction (PID for straight-line hold)
            heading_error = target_heading - self._heading
            diff = self.heading_pid.update(heading_error, dt_actual)

            left_vel, left_pwm = self.left.update(vel - diff)
            right_vel, right_pwm = self.right.update(vel + diff)

            self._update_position(left_vel, right_vel, omega, dt_actual)
            self._log(t, left_vel, right_vel, left_pwm, right_pwm, heading_error, diff)

            t += dt_actual
            elapsed = monotonic() - now
            if elapsed < DT:
                sleep(DT - elapsed)

        self.stop()

    def turn(self, degrees: float, velocity: float = MAX_TURN_VELOCITY) -> None:
        """Turn in place with trapezoidal velocity profile.

        The velocity profile handles the gross motion (ramp up → cruise →
        ramp down based on angle remaining). The heading PID provides only
        fine correction for the last few degrees.

        Args:
            degrees: Rotation angle (positive = counterclockwise).
            velocity: Peak wheel speed in ticks/sec.
        """
        if abs(degrees) < 1.0:
            return  # Too small to bother

        self.left.reset()
        self.right.reset()
        self.heading_pid.reset()

        target_heading = self._heading + degrees
        total_angle = abs(degrees)
        direction = 1.0 if degrees > 0 else -1.0

        t = 0.0
        last_time = monotonic()
        while abs(target_heading - self._heading) > 1.0:
            now = monotonic()
            dt_actual = now - last_time
            last_time = now

            omega = self._update_heading(dt_actual)

            # How far we've turned so far
            angle_traveled = total_angle - abs(target_heading - self._heading)
            angle_traveled = max(0.0, angle_traveled)

            # Trapezoidal velocity profile (primary controller)
            factor = self._speed_factor(
                angle_traveled, total_angle, RAMP_ANGLE_DEG, MIN_TURN_FACTOR
            )
            profiled_vel = direction * velocity * factor

            # Fine correction PID (small adjustments only)
            heading_error = target_heading - self._heading
            fine_correction = self.heading_pid.update(heading_error, dt_actual)
            fine_correction *= TURN_PID_FINE_GAIN

            # Combine: profile drives motion, PID fine-tunes
            turn_vel = profiled_vel + fine_correction

            # Differential drive: left backward, right forward (for CCW)
            left_vel, left_pwm = self.left.update(-turn_vel)
            right_vel, right_pwm = self.right.update(turn_vel)

            self._update_position(left_vel, right_vel, omega, dt_actual)
            self._log(
                t,
                left_vel,
                right_vel,
                left_pwm,
                right_pwm,
                heading_error,
                fine_correction,
            )

            t += dt_actual
            elapsed = monotonic() - now
            if elapsed < DT:
                sleep(DT - elapsed)

        self.stop()

    def move_to(self, target_x: float, target_y: float) -> None:
        """Move to a target point: turn to face it, then drive straight.

        Args:
            target_x: Target X position in cm (world frame).
            target_y: Target Y position in cm (world frame).
        """
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        if dist < 1.0:
            return  # Already there

        # Compute heading to target
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = (target_heading - self._heading + 180) % 360 - 180

        # Turn to face target (skip if nearly aligned)
        if abs(heading_error) > 2.0:
            self.turn(heading_error)

        # Drive forward to target
        self.forward(dist)

    def follow_path(
        self,
        waypoints: list[tuple[float, float]],
        max_distance_cm: float = 0,
    ) -> int:
        """Follow a path of world-coordinate waypoints.

        For each waypoint: turn to face it, then drive straight.
        If max_distance_cm > 0, stops after reaching the first waypoint
        that exceeds the cumulative distance threshold (for scan breaks).

        Args:
            waypoints: List of (x, y) in world cm. First is current position.
            max_distance_cm: If > 0, stop after this cumulative distance.
                             0 means follow entire path.

        Returns:
            Index of the last waypoint reached (1-based, into the waypoints
            list). Can be used to resume from the remaining waypoints.
        """
        if len(waypoints) < 2:
            return 0

        cumulative = 0.0
        last_x, last_y = self.x, self.y

        for i, (wx, wy) in enumerate(waypoints[1:], start=1):
            self.move_to(wx, wy)

            # Track cumulative distance
            dx = self.x - last_x
            dy = self.y - last_y
            cumulative += math.hypot(dx, dy)
            last_x, last_y = self.x, self.y

            # Stop for scan if distance budget exceeded
            if max_distance_cm > 0 and cumulative >= max_distance_cm:
                return i

        return len(waypoints) - 1

    # ── Pose Management ────────────────────────────────────────────────

    def get_pose(self) -> tuple[float, float, float]:
        """Return current pose (x, y, heading) in cm/degrees."""
        return (self.x, self.y, self._heading)

    def set_pose(self, x: float, y: float, heading_deg: float) -> None:
        """Set pose externally (e.g. after ICP correction)."""
        self.x = x
        self.y = y
        self._heading = heading_deg

    def reset_pose(self) -> None:
        """Reset pose to origin."""
        self._heading = 0.0
        self.x = 0.0
        self.y = 0.0

    def stop(self) -> None:
        """Stop both motors."""
        self.left.stop()
        self.right.stop()

    def close(self) -> None:
        """Release resources."""
        self.stop()
        self.left.close()
        self.right.close()
