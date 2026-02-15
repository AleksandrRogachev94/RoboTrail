import math
import time

import adafruit_vl53l1x
import board
import matplotlib.pyplot as plt
import numpy as np

from robot.config import SERVO_ANGLE_OFFSET
from sensors.servo import HWServo


# Default parameters are optimal calibrated values
class Scanner:
    def __init__(
        self,
        num_points=40,
        timing_budget=100,
        settle_delay=0.01,
        distance_mode=2,
        servo_reversed=True,
    ):
        print("initializing scanner")
        self.num_points = num_points
        self.settle_delay = settle_delay

        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.vl53 = adafruit_vl53l1x.VL53L1X(self.i2c)
        self.vl53.distance_mode = distance_mode
        self.vl53.timing_budget = timing_budget

        self.servo = HWServo(
            channel=2, reversed=servo_reversed, angle_offset=SERVO_ANGLE_OFFSET
        )

    def scan(self, from_angle=-60, to_angle=60):
        """
        Scan from from_angle to to_angle.

        Args:
            from_angle: Starting angle in degrees (default -60)
            to_angle: Ending angle in degrees (default 60)

        Returns:
            numpy array of shape (N, 2) with x, y coordinates in cm
        """
        self.servo.set_angle(from_angle)
        self.vl53.start_ranging()
        time.sleep(0.5)

        angles = np.linspace(from_angle, to_angle, num=self.num_points)
        distances = []

        for angle in angles:
            self.servo.set_angle(angle)
            time.sleep(self.settle_delay)

            # Discard stale reading (captured during servo motion)
            while not self.vl53.data_ready:
                time.sleep(0.001)
            self.vl53.clear_interrupt()

            # Capture fresh reading (started after servo settled)
            while not self.vl53.data_ready:
                time.sleep(0.001)
            distances.append(self.vl53.distance)
            self.vl53.clear_interrupt()

        self.vl53.stop_ranging()

        cartesian = []
        for i, angle in enumerate(angles):
            angle_rad = math.radians(angle)
            distance = distances[i]
            if distance is None:
                continue
            # Convert to Cartesian (Robot Frame: X=Forward, Y=Left)
            # Angle 0 is Forward (+X)
            # Note: Y is inverted because physical servo moves Right for +Angle commands
            x = distance * math.cos(angle_rad)
            y = -distance * math.sin(angle_rad)
            cartesian.append([x, y])

        return np.array(cartesian)


if __name__ == "__main__":
    scan = Scanner().scan()

    # Create the scatter plot
    plt.scatter(scan[:, 0], scan[:, 1])

    # Add title and labels (optional, but recommended)
    plt.title("TOF Scan")
    plt.xlabel("X, cm")
    plt.ylabel("Y, cm")
    plt.tight_layout()
    plt.savefig("test_scan.png", dpi=150, bbox_inches="tight")
