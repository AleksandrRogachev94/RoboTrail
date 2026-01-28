"""Hardware PWM servo control via sysfs (Pi 5 compatible)."""


class HWServo:
    def __init__(self, channel=2, chip=0, reversed=False, angle_offset=0.0):
        """
        Args:
            channel: PWM channel (2 for GPIO 18)
            chip: PWM chip number
            reversed: If True, negate angles (for flipped servo mounting)
            angle_offset: Calibration offset to correct servo 0° to true forward
        """
        self.reversed = reversed
        self.angle_offset = angle_offset

        # Path to PWM control files in Linux sysfs
        # /sys/class/pwm/pwmchip0/pwm2/ for GPIO 18
        self.path = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
        # Tell the kernel to create the PWM channel interface
        # This creates /sys/class/pwm/pwmchip0/pwm2/ directory
        try:
            with open(f"/sys/class/pwm/pwmchip{chip}/export", "w") as f:
                f.write(str(channel))
        except IOError:
            pass  # Already exported from previous run
        # Set the period: 20,000,000 nanoseconds = 20ms = 50Hz
        # Servos expect exactly this frequency
        with open(f"{self.path}/period", "w") as f:
            f.write("20000000")
        # Turn on the PWM output
        with open(f"{self.path}/enable", "w") as f:
            f.write("1")

    def set_angle(self, angle):
        """Set servo angle from -90 (left) to +90 (right), 0 = forward."""
        # Apply reversal if configured (for flipped servo mounting)
        if self.reversed:
            angle = -angle

        # Apply calibration offset and clamp to physical servo limits
        angle += self.angle_offset
        angle = max(-90, min(90, angle))

        # Convert angle (-90 to +90) to pulse width in nanoseconds
        # -90° → 500,000 ns  (0.5ms) - full left
        #   0° → 1,500,000 ns (1.5ms) - center/forward
        # +90° → 2,500,000 ns (2.5ms) - full right
        pulse_ns = int(1500000 + (angle / 90) * 1000000)
        # Write the duty cycle (pulse width)
        with open(f"{self.path}/duty_cycle", "w") as f:
            f.write(str(pulse_ns))
