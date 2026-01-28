"""Calibrate servo 0° angle offset using ToF distance to a perpendicular wall.

Place robot perpendicular to a wall, run this script.
The peak distance indicates true 0° — the offset is printed and plotted.
"""

import time

import adafruit_vl53l1x
import board
import matplotlib.pyplot as plt
import numpy as np

from sensors.servo import HWServo


def calibrate(sweep_range=15, step=0.5, num_sweeps=3):
    """Sweep servo and find true 0° using parabolic fit."""
    i2c = board.I2C()
    tof = adafruit_vl53l1x.VL53L1X(i2c)
    tof.timing_budget = 100
    servo = HWServo(channel=2, reversed=True)

    angles = np.arange(-sweep_range, sweep_range + step, step)
    all_distances = []

    servo.set_angle(-sweep_range)
    tof.start_ranging()
    time.sleep(0.5)

    for sweep in range(num_sweeps):
        print(f"Sweep {sweep + 1}/{num_sweeps}")
        distances = []
        for angle in angles:
            servo.set_angle(angle)
            time.sleep(0.05)
            while not tof.data_ready:
                time.sleep(0.001)
            tof.clear_interrupt()
            while not tof.data_ready:
                time.sleep(0.001)
            distances.append(tof.distance or 0)
            tof.clear_interrupt()
        all_distances.append(distances)

    tof.stop_ranging()
    servo.set_angle(0)

    # Average across sweeps
    avg_distances = np.mean(all_distances, axis=0)

    # Parabolic fit: d = a*θ² + b*θ + c, peak at θ = -b/(2a)
    coeffs = np.polyfit(angles, avg_distances, deg=2)
    offset = -coeffs[1] / (2 * coeffs[0])
    fit_curve = np.polyval(coeffs, angles)

    print("\n=== RESULT ===")
    print(f"Servo offset: {offset:.2f}°")
    print(f"Add to config.py: SERVO_ANGLE_OFFSET = {offset:.2f}")

    # Plot
    plt.figure(figsize=(8, 5))
    plt.scatter(angles, avg_distances, label="Measurements", zorder=5)
    plt.plot(angles, fit_curve, "r-", label="Parabolic fit")
    plt.axvline(offset, color="g", linestyle="--", label=f"Peak: {offset:.2f}°")
    plt.xlabel("Servo Angle (°)")
    plt.ylabel("Distance (cm)")
    plt.title("Servo Angle Calibration")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig("servo_angle_calibration.png", dpi=150)
    print("Saved plot: servo_angle_calibration.png")

    return offset


if __name__ == "__main__":
    calibrate()
