"""DC motor control via TB6612FNG driver."""

import lgpio

from robot.config import DC_PWM_FREQ


class DCMotor:
    """Controls a single DC motor via TB6612FNG H-bridge."""

    def __init__(self, chip: int, pwm_pin: int, in1_pin: int, in2_pin: int):
        """
        Initialize DC motor.

        Args:
            chip: lgpio chip handle from gpiochip_open()
            pwm_pin: GPIO pin for PWM speed control
            in1_pin: GPIO pin for direction control (IN1)
            in2_pin: GPIO pin for direction control (IN2)
        """
        self.chip = chip
        self.pwm_pin = pwm_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin

        # Claim pins as outputs
        for pin in [pwm_pin, in1_pin, in2_pin]:
            lgpio.gpio_claim_output(chip, pin, 0)

    def set_speed(self, speed: float) -> None:
        """
        Set motor speed and direction.

        Args:
            speed: -100 to 100 (negative = reverse)
        """
        if speed > 0:
            lgpio.gpio_write(self.chip, self.in1_pin, 1)
            lgpio.gpio_write(self.chip, self.in2_pin, 0)
        elif speed < 0:
            lgpio.gpio_write(self.chip, self.in1_pin, 0)
            lgpio.gpio_write(self.chip, self.in2_pin, 1)
        else:
            lgpio.gpio_write(self.chip, self.in1_pin, 0)
            lgpio.gpio_write(self.chip, self.in2_pin, 0)

        duty = min(abs(speed), 100)
        lgpio.tx_pwm(self.chip, self.pwm_pin, DC_PWM_FREQ, duty)

    def stop(self) -> None:
        """Stop motor."""
        lgpio.tx_pwm(self.chip, self.pwm_pin, DC_PWM_FREQ, 0)
        lgpio.gpio_write(self.chip, self.in1_pin, 0)
        lgpio.gpio_write(self.chip, self.in2_pin, 0)
