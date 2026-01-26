#!/usr/bin/env python3
"""Test PID velocity control on one motor.

Run on Pi: python3 pid_motor_test.py
"""

import time

import lgpio

from robot.config import DC_RIGHT_IN1, DC_RIGHT_IN2, DC_RIGHT_PWM
from robot.dc_motor import DCMotor
from robot.pid import PID
from sensors.encoder import Encoder, list_encoder_devices

# Config
TARGET_VELOCITY = 300  # ticks/sec - adjust based on your encoder!
DT = 0.02  # 50 Hz
DURATION = 5.0  # seconds


def main():
    # 1. Setup GPIO and motor
    h = lgpio.gpiochip_open(4)
    motor = DCMotor(h, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)

    # 2. Setup encoder
    encoders = list_encoder_devices()
    if not encoders:
        print("No encoder devices found!")
        lgpio.gpiochip_close(h)
        return

    print(f"Found encoders: {encoders}")
    # Create Encoder instance (right encoder is first device)
    enc = Encoder(encoders[0][0])

    # 3. Setup PID - START WITH P ONLY!
    pid = PID(kp=0.5, ki=0.0, kd=0.0)

    # 4. Control loop variables
    # Initialize last_pos, last_time, start_time
    last_pos = 0
    last_time = time.monotonic()
    start_time = last_time

    print(f"\nTarget: {TARGET_VELOCITY} ticks/sec")
    print("Starting in 1 second...")
    time.sleep(1)

    try:
        while (time.monotonic() - start_time) < DURATION:
            # Calculate dt_actual from time difference
            now = time.monotonic()
            dt_actual = now - last_time

            # Read encoder position
            pos = enc.position

            # Calculate velocity = (pos - last_pos) / dt_actual
            velocity = (pos - last_pos) / dt_actual

            # Calculate error = TARGET_VELOCITY - velocity
            error = TARGET_VELOCITY - velocity
            # Get PID output
            output = pid.update(error, dt_actual)

            # Apply to motor using motor.set_speed(output)
            motor.set_speed(output)

            # Print debug info
            print(
                f"vel={velocity:6.0f}  target={TARGET_VELOCITY}  err={error:+6.0f}  pwm={output:5.1f}"
            )

            # Update last_pos, last_time
            last_pos = pos
            last_time = now

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        motor.stop()
        enc.close()
        lgpio.gpiochip_close(h)
        print("Done!")


if __name__ == "__main__":
    main()
