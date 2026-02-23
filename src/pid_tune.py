#!/usr/bin/env python3
"""PID tuning script for DC motor velocity controller.

Ramps target velocity up (like the real velocity profile), cruises,
then ramps down — so the PID is tested under realistic conditions.

Both motors run simultaneously with their own PID+feedforward loops
for realistic chassis load.

Run on Pi:
    python3 pid_tune.py --motor right --kp 0.15
    python3 pid_tune.py --motor left  --kp 0.15 --ki 0.1

Tuning procedure:
  1. Place robot on the floor it will normally navigate.
  2. Start with kp only (ki=0, kd=0).  Look at the cruise phase —
     velocity should track the red target line closely (within 30 ticks/s).
  3. Add small ki (0.01–0.05) to eliminate any steady-state offset
     visible in the cruise phase.
  4. Update PID_KP / PID_KI / PID_KD in config.py when satisfied.
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

# ── Config ────────────────────────────────────────────────────────────
CRUISE_VELOCITY = 700  # ticks/sec — match MAX_FORWARD_VELOCITY
DT = 0.02  # 50 Hz — match real control loop
EMA_ALPHA = 0.2  # Must match DCMotorPID._EMA_ALPHA

# Profile timing (total = RAMP + CRUISE)
RAMP_TIME = 1.0  # seconds to ramp 0→cruise
CRUISE_TIME = 4.0  # seconds at full speed


def feedforward(target_velocity: float) -> float:
    """Identical to DCMotorPID._feedforward."""
    if target_velocity == 0:
        return 0
    sign = 1 if target_velocity > 0 else -1
    return sign * (FEEDFORWARD_OFFSET + FEEDFORWARD_SLOPE * abs(target_velocity))


def get_target(t: float) -> float:
    """Trapezoidal velocity profile: ramp up → cruise."""
    if t < RAMP_TIME:
        return CRUISE_VELOCITY * (t / RAMP_TIME)
    return CRUISE_VELOCITY


def run_motor(motor, enc, pid, last_pos, ema_vel, dt, target):
    """One PID+feedforward iteration."""
    pos = enc.position
    raw_vel = (pos - last_pos) / dt if dt > 0 else 0
    ema = EMA_ALPHA * raw_vel + (1 - EMA_ALPHA) * ema_vel
    ff = feedforward(target)
    error = target - ema
    correction = pid.update(error, dt)
    output = max(-100, min(100, ff + correction))
    motor.set_speed(output)
    return pos, ema, output


def main():
    parser = argparse.ArgumentParser(description="PID tuning — wheel velocity")
    parser.add_argument("--motor", choices=["left", "right"], required=True)
    parser.add_argument("--kp", type=float, default=PID_KP)
    parser.add_argument("--ki", type=float, default=PID_KI)
    parser.add_argument("--kd", type=float, default=PID_KD)
    args = parser.parse_args()

    # ── Hardware ──────────────────────────────────────────────────────
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

    if args.motor == "right":
        tune_motor, tune_enc = right_motor, right_enc
        companion_motor, companion_enc = left_motor, left_enc
    else:
        tune_motor, tune_enc = left_motor, left_enc
        companion_motor, companion_enc = right_motor, right_enc

    tune_pid = PID(kp=args.kp, ki=args.ki, kd=args.kd)
    companion_pid = PID(kp=PID_KP, ki=PID_KI, kd=PID_KD)

    # ── Data ──────────────────────────────────────────────────────────
    times, target_list, tune_vels, companion_vels, pwms = [], [], [], [], []

    tune_pos = tune_enc.position
    companion_pos = companion_enc.position
    tune_ema = 0.0
    companion_ema = 0.0
    duration = RAMP_TIME + CRUISE_TIME

    print(f"\nTuning {args.motor} motor")
    print(f"  Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print(f"  Profile: ramp {RAMP_TIME}s -> cruise {CRUISE_TIME}s at {CRUISE_VELOCITY}")
    print(f"  Companion: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    print("\nPlace robot on floor, press Enter to start...")
    input()

    last_time = time.monotonic()
    start_time = last_time

    try:
        while (time.monotonic() - start_time) < duration:
            now = time.monotonic()
            dt_actual = now - last_time
            if dt_actual <= 0:
                dt_actual = DT

            t = now - start_time
            target = get_target(t)

            tune_pos, tune_ema, pwm = run_motor(
                tune_motor, tune_enc, tune_pid, tune_pos, tune_ema, dt_actual, target
            )
            companion_pos, companion_ema, _ = run_motor(
                companion_motor,
                companion_enc,
                companion_pid,
                companion_pos,
                companion_ema,
                dt_actual,
                target,
            )

            times.append(t)
            target_list.append(target)
            tune_vels.append(tune_ema)
            companion_vels.append(companion_ema)
            pwms.append(pwm)

            print(
                f"t={t:4.2f}  target={target:5.0f}  tune={tune_ema:6.0f}"
                f"  err={target - tune_ema:+6.0f}  pwm={pwm:5.1f}"
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
        ax1.plot(times, target_list, "r--", label="target")
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
