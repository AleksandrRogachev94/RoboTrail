import time

import adafruit_vl53l1x
import board
import matplotlib.pyplot as plt
import numpy as np

from sensors.servo import HWServo

servo = HWServo(channel=2, reversed=True)

i2c = board.I2C()  # uses board.SCL and board.SDA
vl53 = adafruit_vl53l1x.VL53L1X(i2c)
vl53.distance_mode = 2
vl53.timing_budget = 50
print(f"Current TOF Mode: {'Short' if vl53.distance_mode == 1 else 'Long'}")


# Default values are optimal
def scan(num_points=30, timing_budget=50, settle_delay=0.01):
    """Run a single scan with specified parameters."""
    vl53.timing_budget = timing_budget

    servo.set_angle(-90)
    vl53.start_ranging()
    time.sleep(0.5)

    angles = np.linspace(-90, 90, num=num_points)
    distances = []

    for angle in angles:
        servo.set_angle(angle)
        time.sleep(settle_delay)

        while not vl53.data_ready:
            time.sleep(0.001)

        distances.append(vl53.distance)
        vl53.clear_interrupt()

    vl53.stop_ranging()
    return angles, distances


def plot_comparison(results, title, filename):
    """Plot multiple scans on one polar chart."""
    plt.figure(figsize=(12, 10))
    ax = plt.subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_thetamin(-90)
    ax.set_thetamax(90)

    colors = ["red", "blue", "green", "orange", "purple", "cyan"]
    for (angles, distances, label), color in zip(results, colors):
        angles_rad = np.deg2rad(angles)
        ax.plot(
            angles_rad,
            distances,
            "o-",
            label=label,
            color=color,
            markersize=4,
            alpha=0.7,
        )

    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.0))
    plt.title(title)
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    print(f"Saved to {filename}")


# ============================================================
# TEST 1: Vary timing budget (with fixed points and delay)
# ============================================================
def test_timing_budget():
    configs = [
        (30, 20, 0, "20ms"),
        (30, 33, 0, "33ms"),
        (30, 50, 0, "50ms"),
        (30, 100, 0, "100ms"),
    ]
    results = []
    for pts, budget, delay, label in configs:
        print(f"Scanning: {label}")
        start = time.time()
        angles, distances = scan(pts, budget, delay)
        elapsed = time.time() - start
        results.append((angles, distances, f"{label} ({elapsed:.1f}s)"))

    plot_comparison(results, "Timing Budget Comparison", "test_timing_budget.png")


# ============================================================
# TEST 2: Vary number of points (with fixed budget and delay)
# ============================================================
def test_num_points():
    configs = [
        (15, 50, 0, "15 pts"),
        (20, 50, 0, "20 pts"),
        (30, 50, 0, "30 pts"),
        (45, 50, 0, "45 pts"),
    ]
    results = []
    for pts, budget, delay, label in configs:
        print(f"Scanning: {label}")
        start = time.time()
        angles, distances = scan(pts, budget, delay)
        elapsed = time.time() - start
        results.append((angles, distances, f"{label} ({elapsed:.1f}s)"))

    plot_comparison(results, "Number of Points Comparison", "test_num_points.png")


# ============================================================
# TEST 3: Vary servo settle delay (with fixed budget and points)
# ============================================================
def test_settle_delay():
    configs = [
        (30, 50, 0.00, "0ms"),
        (30, 50, 0.01, "10ms"),
        (30, 50, 0.02, "20ms"),
        (30, 50, 0.05, "50ms"),
    ]
    results = []
    for pts, budget, delay, label in configs:
        print(f"Scanning: {label}")
        start = time.time()
        angles, distances = scan(pts, budget, delay)
        elapsed = time.time() - start
        results.append((angles, distances, f"{label} ({elapsed:.1f}s)"))

    plot_comparison(results, "Settle Delay Comparison", "test_settle_delay.png")


# ============================================================
# Run whichever test you want
# ============================================================
if __name__ == "__main__":
    # Uncomment the test you want to run:

    test_timing_budget()
    test_num_points()
    test_settle_delay()
