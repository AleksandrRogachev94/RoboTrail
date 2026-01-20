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
        plt.plot(
            scan_points[:, 0],
            scan_points[:, 1],
            "o-",
            label=label,
            color=color,
            markersize=4,
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
        {"timing_budget": 33, "label": "33ms"},
        {"timing_budget": 50, "label": "50ms"},
        {"timing_budget": 100, "label": "100ms"},
        {"timing_budget": 200, "label": "200ms"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(timing_budget=cfg["timing_budget"])
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
        {"num_points": 20, "label": "20 pts"},
        {"num_points": 30, "label": "30 pts"},
        {"num_points": 40, "label": "40 pts"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(num_points=cfg["num_points"])
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
        {"settle_delay": 0.05, "label": "50ms"},
        {"settle_delay": 0.1, "label": "100ms"},
        {"settle_delay": 0.2, "label": "200ms"},
    ]
    results = []
    for cfg in configs:
        print(f"Scanning: {cfg['label']}")
        scanner = Scanner(settle_delay=cfg["settle_delay"])
        start = time.time()
        scan_points = scanner.scan()
        elapsed = time.time() - start
        results.append((scan_points, f"{cfg['label']} ({elapsed:.1f}s)"))

    plot_comparison_cartesian(
        results, "Settle Delay Comparison", "test_settle_delay.png"
    )


# ============================================================
# TEST 4: Compare forward vs backward scan direction
# If they match, settle_delay is sufficient
# ============================================================
def test_direction(settle_delay=None):
    """Compare forward and backward scans to verify servo settle time."""
    # Use provided settle_delay or Scanner default
    if settle_delay is not None:
        scanner = Scanner(settle_delay=settle_delay)
    else:
        scanner = Scanner()
        settle_delay = scanner.settle_delay

    print(f"Testing direction with settle_delay={settle_delay}s")

    # Forward scan (-60 to +60)
    print("Forward scan...")
    start = time.time()
    forward = scanner.scan(from_angle=-60, to_angle=60)
    forward_time = time.time() - start

    # Backward scan (+60 to -60)
    print("Backward scan...")
    start = time.time()
    backward = scanner.scan(from_angle=60, to_angle=-60)
    backward_time = time.time() - start

    results = [
        (forward, f"Forward ({forward_time:.1f}s)"),
        (backward, f"Backward ({backward_time:.1f}s)"),
    ]
    plot_comparison_cartesian(
        results, f"Direction Test (settle={settle_delay}s)", "test_direction.png"
    )


# ============================================================
# Run whichever test you want
# ============================================================
if __name__ == "__main__":
    # Uncomment the test you want to run:

    # test_timing_budget()
    # test_num_points()
    # test_settle_delay()
    test_direction()
