"""Calibrate ToF/servo scanning parameters.

Uses the Scanner class to test different configurations.
"""

import time

import matplotlib.pyplot as plt

from scanner import Scanner


def plot_comparison_cartesian(results, title, filename):
    """Plot multiple scans on one Cartesian chart."""
    plt.figure(figsize=(10, 8))

    colors = ["red", "blue", "green", "orange", "purple", "cyan"]
    for (scan_points, label), color in zip(results, colors):
        plt.scatter(
            scan_points[:, 0],
            scan_points[:, 1],
            label=label,
            color=color,
            s=30,
            alpha=0.7,
        )

    plt.grid(True, alpha=0.3)
    plt.legend(loc="upper right")
    plt.title(title)
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches="tight")
    print(f"Saved to {filename}")


# ============================================================
# TEST 1: Vary timing budget (with fixed points and delay)
# ============================================================
def test_timing_budget():
    configs = [
        {"timing_budget": 20, "label": "20ms"},
        {"timing_budget": 33, "label": "33ms"},
        {"timing_budget": 50, "label": "50ms"},
        {"timing_budget": 100, "label": "100ms"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(
            num_points=20,
            timing_budget=cfg["timing_budget"],
            settle_delay=0,
            servo_reversed=True,
        )
        start = time.time()
        scan_points = scanner.scan()
        elapsed = time.time() - start
        results.append((scan_points, f"{cfg['label']} ({elapsed:.1f}s)"))

    plot_comparison_cartesian(
        results, "Timing Budget Comparison", "test_timing_budget.png"
    )


# ============================================================
# TEST 2: Vary number of points (with fixed budget and delay)
# ============================================================
def test_num_points():
    configs = [
        {"num_points": 10, "label": "10 pts"},
        {"num_points": 15, "label": "15 pts"},
        {"num_points": 20, "label": "20 pts"},
        {"num_points": 30, "label": "30 pts"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(
            num_points=cfg["num_points"],
            timing_budget=50,
            settle_delay=0,
            servo_reversed=True,
        )
        start = time.time()
        scan_points = scanner.scan()
        elapsed = time.time() - start
        results.append((scan_points, f"{cfg['label']} ({elapsed:.1f}s)"))

    plot_comparison_cartesian(
        results, "Number of Points Comparison", "test_num_points.png"
    )


# ============================================================
# TEST 3: Vary servo settle delay (with fixed budget and points)
# ============================================================
def test_settle_delay():
    configs = [
        {"settle_delay": 0.00, "label": "0ms"},
        {"settle_delay": 0.01, "label": "10ms"},
        {"settle_delay": 0.02, "label": "20ms"},
        {"settle_delay": 0.05, "label": "50ms"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(
            num_points=20,
            timing_budget=50,
            settle_delay=cfg["settle_delay"],
            servo_reversed=True,
        )
        start = time.time()
        scan_points = scanner.scan()
        elapsed = time.time() - start
        results.append((scan_points, f"{cfg['label']} ({elapsed:.1f}s)"))

    plot_comparison_cartesian(
        results, "Settle Delay Comparison", "test_settle_delay.png"
    )


# ============================================================
# Run whichever test you want
# ============================================================
if __name__ == "__main__":
    # Uncomment the test you want to run:

    test_timing_budget()
    test_num_points()
    test_settle_delay()
