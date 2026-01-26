class PID:
    """Generic PID controller."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_min: float = -100,
        output_max: float = 100,
    ):
        """Store gains and limits."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = output_max / ki
        self.reset()

    def update(self, error: float, dt: float) -> float:
        """Compute PID output given current error.

        Args:
            error: target - actual (positive = need to increase)
            dt: time since last update in seconds

        Returns:
            Control output (clamped to min/max)
        """
        self.integral += error * dt
        self.integral = max(
            -self.integral_limit, min(self.integral, self.integral_limit)
        )
        if self.prev_error is None:
            derivative = 0  # Skip D term on first call
        else:
            derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(
            self.output_min, min(output, self.output_max)
        )  # Clamp to output limits
        self.prev_error = error  # Store prev_error for next iteration
        return output

    def reset(self):
        """Reset integral and prev_error (call when starting new movement)."""
        self.integral = 0
        self.prev_error = None
