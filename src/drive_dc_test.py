#!/usr/bin/env python3
"""Test script for DC motor drive system with plotting.

Use this to:
1. Test forward/backward movement
2. Test turning
3. Calibrate TICKS_PER_CM
4. View PID performance plots

Run on Pi: python3 drive_dc_test.py
"""

import lgpio

from robot.drive_dc import RobotDC


def plot_history(history: list, title: str, filename: str) -> None:
    """Generate and save a plot from movement history."""
    if not history:
        print("No history to plot")
        return

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        times = [h["t"] for h in history]
        left_vel = [h["left_vel"] for h in history]
        right_vel = [h["right_vel"] for h in history]
        heading = [h["heading"] for h in history]
        heading_err = [h["heading_error"] for h in history]

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

        # Velocities
        axes[0].plot(times, left_vel, "b-", label="Left Vel")
        axes[0].plot(times, right_vel, "r-", label="Right Vel")
        axes[0].set_ylabel("Velocity (ticks/s)")
        axes[0].legend()
        axes[0].grid(True)

        # Heading
        axes[1].plot(times, heading, "g-", label="Heading")
        axes[1].plot(times, heading_err, "m--", label="Heading Error")
        axes[1].set_ylabel("Degrees")
        axes[1].set_xlabel("Time (s)")
        axes[1].legend()
        axes[1].grid(True)

        fig.suptitle(title)
        plt.tight_layout()
        plt.savefig(filename, dpi=100)
        print(f"Plot saved: {filename}")

    except ImportError:
        print("Matplotlib not available, skipping plot")


def main():
    print("Initializing robot...")
    chip = lgpio.gpiochip_open(4)
    robot = RobotDC(chip)

    try:
        # Test 1: Forward 30cm
        print("\n=== Test: Forward 30cm ===")
        print("Place robot at a marked starting point.")
        input("Press Enter to start...")

        robot.history.clear()
        robot.forward(30)

        print("Done! Measure actual distance traveled.")
        plot_history(robot.history, "Forward 30cm", "forward_30cm.png")

        actual = input("Enter actual distance in cm (or press Enter to skip): ")
        if actual:
            actual = float(actual)
            correction = 30 / actual
            print(f"Correction factor: {correction:.3f}")
            print(f"New TICKS_PER_CM = {50 * correction:.1f}")

        # Test 2: Turn 90 degrees
        print("\n=== Test: Turn 90 degrees CCW ===")
        input("Press Enter to turn...")

        robot.history.clear()
        robot.turn(90)

        print("Done! Check if it turned ~90 degrees.")
        plot_history(robot.history, "Turn 90 degrees", "turn_90deg.png")

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        robot.close()
        lgpio.gpiochip_close(chip)
        print("Cleanup complete.")


if __name__ == "__main__":
    main()
