#!/usr/bin/env python3
"""Debug DC motor wiring - test each pin individually.

Use a multimeter to verify each GPIO is outputting correctly.
"""

import lgpio

# Pin assignments
LEFT_PWM = 12
RIGHT_PWM = 16
LEFT_IN1 = 20
LEFT_IN2 = 21
RIGHT_IN1 = 24
RIGHT_IN2 = 25

ALL_PINS = [LEFT_PWM, RIGHT_PWM, LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2]
PIN_NAMES = {
    12: "LEFT_PWM (PWMA)",
    16: "RIGHT_PWM (PWMB)",
    20: "LEFT_IN1 (AIN1)",
    21: "LEFT_IN2 (AIN2)",
    24: "RIGHT_IN1 (BIN1)",
    25: "RIGHT_IN2 (BIN2)",
}


def main():
    h = lgpio.gpiochip_open(4)  # Pi 5 uses chip 4

    # Claim all pins as output
    for pin in ALL_PINS:
        lgpio.gpio_claim_output(h, pin, 0)

    print("=" * 50)
    print("DC Motor Debug - Testing each GPIO pin")
    print("Use multimeter between each GPIO and GND")
    print("=" * 50)
    print()

    # Test 1: Set each pin HIGH one at a time
    for pin in ALL_PINS:
        name = PIN_NAMES[pin]
        print(f"Setting GPIO {pin} ({name}) HIGH...")
        print(f"  -> Measure between GPIO {pin} and GND, should read ~3.3V")
        lgpio.gpio_write(h, pin, 1)
        input("  Press Enter when ready for next pin...")
        lgpio.gpio_write(h, pin, 0)

    print()
    print("=" * 50)
    print("Test 2: PWM output test")
    print("=" * 50)

    # Test 2: PWM on motor pins
    print(f"\nStarting PWM on GPIO {LEFT_PWM} at 50% duty cycle...")
    print(f"  -> Measure between GPIO {LEFT_PWM} and GND")
    print("  -> Multimeter should show ~1.6V (average of PWM)")
    lgpio.tx_pwm(h, LEFT_PWM, 1000, 50)
    input("Press Enter to continue...")
    lgpio.tx_pwm(h, LEFT_PWM, 1000, 0)

    print(f"\nStarting PWM on GPIO {RIGHT_PWM} at 50% duty cycle...")
    lgpio.tx_pwm(h, RIGHT_PWM, 1000, 50)
    input("Press Enter to continue...")
    lgpio.tx_pwm(h, RIGHT_PWM, 1000, 0)

    print()
    print("=" * 50)
    print("Test 3: Full motor test - LEFT motor")
    print("=" * 50)
    print("\nSetting LEFT motor forward:")
    print(f"  GPIO {LEFT_IN1} = HIGH")
    print(f"  GPIO {LEFT_IN2} = LOW")
    print(f"  GPIO {LEFT_PWM} = 75% PWM")

    lgpio.gpio_write(h, LEFT_IN1, 1)
    lgpio.gpio_write(h, LEFT_IN2, 0)
    lgpio.tx_pwm(h, LEFT_PWM, 1000, 75)

    print("\n  -> LEFT motor should be spinning!")
    print("  -> If not, check:")
    print("     - A01 connected to motor red wire?")
    print("     - A02 connected to motor white wire?")
    print(f"     - PWMA connected to GPIO {LEFT_PWM}?")
    print(f"     - AIN1 connected to GPIO {LEFT_IN1}?")
    print(f"     - AIN2 connected to GPIO {LEFT_IN2}?")

    input("\nPress Enter to stop and test RIGHT motor...")

    # Stop left
    lgpio.tx_pwm(h, LEFT_PWM, 1000, 0)
    lgpio.gpio_write(h, LEFT_IN1, 0)
    lgpio.gpio_write(h, LEFT_IN2, 0)

    print()
    print("=" * 50)
    print("Test 4: Full motor test - RIGHT motor")
    print("=" * 50)
    print("\nSetting RIGHT motor forward:")
    print(f"  GPIO {RIGHT_IN1} = HIGH")
    print(f"  GPIO {RIGHT_IN2} = LOW")
    print(f"  GPIO {RIGHT_PWM} = 75% PWM")

    lgpio.gpio_write(h, RIGHT_IN1, 1)
    lgpio.gpio_write(h, RIGHT_IN2, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, 1000, 75)

    print("\n  -> RIGHT motor should be spinning!")

    input("\nPress Enter to stop...")

    # Stop all
    lgpio.tx_pwm(h, LEFT_PWM, 1000, 0)
    lgpio.tx_pwm(h, RIGHT_PWM, 1000, 0)
    for pin in ALL_PINS:
        lgpio.gpio_write(h, pin, 0)

    lgpio.gpiochip_close(h)
    print("\nDone!")


if __name__ == "__main__":
    main()
