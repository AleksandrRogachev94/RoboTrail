#!/usr/bin/env python3
"""PID tuning script for a single DC motor velocity controller.

Runs the motor being tuned with its own PID + feedforward (exactly as
DCMotorPID does in normal operation).  The companion motor runs a separate
independent PID loop at the same target velocity so that the chassis load
is realistic (both wheels on the ground, both spinning).

Run on Pi:
    python3 pid_tune.py --motor right --kp 0.15
    python3 pid_tune.py --motor left  --kp 0.15 --ki 0.1

Tuning procedure:
  1. Place robot on the floor it will normally navigate.
  2. Start with kp only (ki=0, kd=0).  Increase kp until you see fast
     rise with tolerable oscillation (~20% overshoot is fine as a start).
  3. Add ki (try 0.1–0.5) to eliminate steady-state error.
  4. Add kd only if there is persistent oscillation after step 3.
  5. Update PID_KP / PID_KI / PID_KD in config.py when satisfied.
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
    FEEDFORWARD_OFFSET,
    FEEDFORWARD_SLOPE,
    PID_KD,
    PID_KI,
    PID_KP,
)
from robot.dc_motor import DCMotor
from robot.pid import PID
from sensors.encoder import Encoder, list_encoder_devices

# Config
TARGET_VELOCITY = 700  # ticks/sec — match MAX_FORWARD_VELOCITY
DT = 0.02  # 50 Hz — match real control loop
DURATION = 5.0  # seconds
EMA_ALPHA = 0.2  # Must match DCMotorPID._EMA_ALPHA


def feedforward(target_velocity: float) -> float:
    """Identical to DCMotorPID._feedforward."""
    if target_velocity == 0:
        return 0
    sign = 1 if target_velocity > 0 else -1
    return sign * (FEEDFORWARD_OFFSET + FEEDFORWARD_SLOPE * abs(target_velocity))


def run_motor(motor, enc, pid, last_pos, last_time, ema_vel, dt):
    """One PID+feedforward iteration. Returns (new_ema_vel, pwm_output)."""
    pos = enc.position
    raw_vel = (pos - last_pos) / dt
    ema = EMA_ALPHA * raw_vel + (1 - EMA_ALPHA) * ema_vel
    ff = feedforward(TARGET_VELOCITY)
    error = TARGET_VELOCITY - ema
    correction = pid.update(error, dt)
    output = max(-100, min(100, ff + correction))
    motor.set_speed(output)
    return pos, ema, output


def main():
    parser = argparse.ArgumentParser(description="PID tuning — wheel velocity")
    parser.add_argument(
        "--motor", choices=["left", "right"], required=True, help="Motor to tune"
    )
    parser.add_argument("--kp", type=float, default=PID_KP)
    parser.add_argument("--ki", type=float, default=PID_KI)
    parser.add_argument("--kd", type=float, default=PID_KD)
    args = parser.parse_args()

    # ── Hardware setup ────────────────────────────────────────────────
    h = lgpio.gpiochip_open(4)
    left_motor = DCMotor(h, DC_LEFT_PWM, DC_LEFT_IN1, DC_LEFT_IN2)
    right_motor = DCMotor(h, DC_RIGHT_PWM, DC_RIGHT_IN1, DC_RIGHT_IN2)

    encoders = list_encoder_devices()
    if len(encoders) < 2:
        print(f"Expected 2 encoders, found {len(encoders)}")
        lgpio.gpiochip_close(h)
        return

    right_enc = Encoder(encoders[0][0])
    left_enc = Encoder(encoders[1][0], reversed=True)

    # ── Select tune/companion ─────────────────────────────────────────
    if args.motor == "right":
        tune_motor, tune_enc = right_motor, right_enc
        companion_motor, companion_enc = left_motor, left_enc
    else:
        tune_motor, tune_enc = left_motor, left_enc
        companion_motor, companion_enc = right_motor, right_enc

    tune_pid = PID(kp=args.kp, ki=args.ki, kd=args.kd)
    companion_pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)  # config gains

    # ── Data collection ───────────────────────────────────────────────
    times, tune_vels, companion_vels, pwms = [], [], [], []

    tune_pos = tune_enc.position
    companion_pos = companion_enc.position
    tune_ema = 0.0
    companion_ema = 0.0
    last_time = time.monotonic()
    start_time = last_time

    print(f"\nTuning {args.motor} motor")
    print(f"  Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print(f"  Target: {TARGET_VELOCITY} ticks/sec")
    print(f"  Companion uses config gains: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    print(f"  EMA alpha: {EMA_ALPHA}  (matches DCMotorPID)")
    print("\nPlace robot on floor, then press Enter to start...")
    input()

    try:
        while (time.monotonic() - start_time) < DURATION:
            now = time.monotonic()
            dt_actual = now - last_time
            if dt_actual <= 0:
                dt_actual = DT

            tune_pos, tune_ema, pwm = run_motor(
                tune_motor, tune_enc, tune_pid, tune_pos, last_time, tune_ema, dt_actual
            )
            companion_pos, companion_ema, _ = run_motor(
                companion_motor,
                companion_enc,
                companion_pid,
                companion_pos,
                last_time,
                companion_ema,
                dt_actual,
            )

            t = now - start_time
            times.append(t)
            tune_vels.append(tune_ema)
            companion_vels.append(companion_ema)
            pwms.append(pwm)

            print(
                f"t={t:4.2f}  tune={tune_ema:6.0f}  companion={companion_ema:6.0f}"
                f"  err={TARGET_VELOCITY - tune_ema:+6.0f}  pwm={pwm:5.1f}"
            )

            last_time = now
            loop_time = time.monotonic() - now
            if DT - loop_time > 0:
                time.sleep(DT - loop_time)

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        tune_motor.stop()
        companion_motor.stop()
        tune_enc.close()
        companion_enc.close()
        lgpio.gpiochip_close(h)

    # ── Plot ──────────────────────────────────────────────────────────
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        ax1.plot(times, tune_vels, "b-", label=f"{args.motor} (tuning)")
        ax1.plot(times, companion_vels, "g-", alpha=0.5, label="companion")
        ax1.axhline(y=TARGET_VELOCITY, color="r", linestyle="--", label="target")
        ax1.set_ylabel("Velocity (ticks/s)")
        ax1.set_title(
            f"{args.motor.capitalize()} — Kp={args.kp}, Ki={args.ki}, Kd={args.kd}"
        )
        ax1.legend()
        ax1.grid(True)

        ax2.plot(times, pwms, "orange", label="PWM output")
        ax2.set_ylabel("PWM (%)")
        ax2.set_xlabel("Time (s)")
        ax2.legend()
        ax2.grid(True)

        filename = f"pid_tune_{args.motor}_kp{args.kp}.png"
        plt.tight_layout()
        plt.savefig(filename, dpi=100)
        print(f"\nPlot saved: {filename}")
    except ImportError:
        print("\nMatplotlib not available, skipping plot")

    print("Done!")


if __name__ == "__main__":
    main()
