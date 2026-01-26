#!/usr/bin/env python3
"""PID tuning script using Ziegler-Nichols method.

Tunes one motor with PID while the other runs at fixed PWM.
Saves velocity plot for finding oscillation period.

Run on Pi:
    python3 pid_tune.py --motor right --kp 0.3
    python3 pid_tune.py --motor left --kp 0.3
"""

import argparse
import time

import lgpio

from robot.config import (
    DC_LEFT_IN1,
    DC_LEFT_IN2,
    DC_LEFT_PWM,
    DC_RIGHT_IN1,
    DC_RIGHT_IN2,
    DC_RIGHT_PWM,
)
from robot.dc_motor import DCMotor
from robot.pid import PID
from sensors.encoder import Encoder, list_encoder_devices

# Config
TARGET_VELOCITY = 700  # ticks/sec
COMPANION_PWM = 40  # Fixed PWM for the other motor
DT = 0.02  # 50 Hz
DURATION = 5.0  # seconds
VELOCITY_SAMPLES = 1  # Moving average window for smoothing


def main():
    parser = argparse.ArgumentParser(description="PID tuning with Ziegler-Nichols")
    parser.add_argument("--motor", choices=["left", "right"], required=True)
    parser.add_argument("--kp", type=float, required=True)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    args = parser.parse_args()

    # 1. Setup GPIO and motors
    h = lgpio.gpiochip_open(4)
    left_motor = DCMotor(h, DC_LEFT_PWM, DC_LEFT_IN1, DC_LEFT_IN2)
    right_motor = DCMotor(h, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)

    # 2. Setup encoders
    encoders = list_encoder_devices()
    if len(encoders) < 2:
        print(f"Expected 2 encoders, found {len(encoders)}")
        lgpio.gpiochip_close(h)
        return

    # Right encoder is first (GPIO 5/6), Left is second (GPIO 17/27 reversed)
    right_enc = Encoder(encoders[0][0])
    left_enc = Encoder(encoders[1][0], reversed=True)

    # Select which motor to tune
    if args.motor == "right":
        tune_motor = right_motor
        tune_enc = right_enc
        companion_motor = left_motor
    else:
        tune_motor = left_motor
        tune_enc = left_enc
        companion_motor = right_motor

    # 3. Setup PID
    pid = PID(kp=args.kp, ki=args.ki, kd=args.kd)

    # 4. Data collection
    times = []
    velocities = []

    # 5. Control loop
    last_pos = tune_enc.position
    last_time = time.monotonic()
    start_time = last_time
    velocity_history = []  # For smoothing

    print(f"\nTuning {args.motor} motor: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print(f"Target: {TARGET_VELOCITY} ticks/sec, Companion at {COMPANION_PWM}% PWM")
    print("Starting in 1 second...")
    time.sleep(1)

    # Start companion motor at fixed speed
    companion_motor.set_speed(COMPANION_PWM)

    try:
        while (time.monotonic() - start_time) < DURATION:
            now = time.monotonic()
            dt_actual = now - last_time

            # Measure velocity (with smoothing)
            pos = tune_enc.position
            raw_velocity = (pos - last_pos) / dt_actual
            velocity_history.append(raw_velocity)
            if len(velocity_history) > VELOCITY_SAMPLES:
                velocity_history.pop(0)
            velocity = sum(velocity_history) / len(velocity_history)

            # PID
            error = TARGET_VELOCITY - velocity
            output = pid.update(error, dt_actual)

            # Apply
            tune_motor.set_speed(output)

            # Log
            times.append(now - start_time)
            velocities.append(velocity)
            print(
                f"t={now - start_time:4.2f}  vel={velocity:6.0f}  err={error:+6.0f}  pwm={output:5.1f}"
            )

            # Update
            last_pos = pos
            last_time = now

            # Sleep remainder of DT for consistent timing
            loop_time = time.monotonic() - now
            sleep_time = DT - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        tune_motor.stop()
        companion_motor.stop()
        right_enc.close()
        left_enc.close()
        lgpio.gpiochip_close(h)

    # 6. Save plot
    try:
        import matplotlib

        matplotlib.use("Agg")  # No display needed
        import matplotlib.pyplot as plt

        plt.figure(figsize=(10, 4))

        # Skip first 0.5 seconds (startup transient)
        plot_times = [t for t in times if t >= 0]
        plot_vels = [v for t, v in zip(times, velocities) if t >= 0]

        plt.plot(plot_times, plot_vels, "b-", label="Velocity")
        plt.axhline(y=TARGET_VELOCITY, color="r", linestyle="--", label="Target")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (ticks/s)")
        plt.title(
            f"{args.motor.capitalize()} Motor: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}"
        )
        plt.legend()
        plt.grid(True)

        filename = f"pid_tune_{args.motor}_kp{args.kp}.png"
        plt.savefig(filename, dpi=100)
        print(f"\nPlot saved: {filename}")
    except ImportError:
        print("\nMatplotlib not available, skipping plot")

    print("Done!")


if __name__ == "__main__":
    main()
