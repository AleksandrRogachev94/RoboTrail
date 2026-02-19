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
        settle_delay=0.03,
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

    def scan(self, from_angle=-90, to_angle=90):
        """
        Scan from from_angle to to_angle.

        Args:
            from_angle: Starting angle in degrees (default -60)
            to_angle: Ending angle in degrees (default 60)

        Returns:
            numpy array of shape (N, 2) with x, y coordinates in cm
        """
        # Pre-position servo and wait for it to fully settle.
        # The servo may need to travel up to 180° from the end of
        # the previous scan; at ~60°/60ms that takes ~300ms.
        self.servo.set_angle(from_angle)
        self.vl53.stop_ranging()
        time.sleep(0.5)
        # Clear any stale data_ready flag left over from the previous scan's
        # stop_ranging(). Without this, i=0's start_ranging() sees data_ready=True
        # immediately and reads the old distance from the previous servo position.
        self.vl53.clear_interrupt()

        angle_step = (to_angle - from_angle) / (self.num_points - 1)
        distances = []
        angles = []

        for i in range(self.num_points):
            angle = from_angle + i * angle_step
            angles.append(angle)
            self.servo.set_angle(angle)
            time.sleep(
                self.settle_delay
            )  # settle after each small step (~4.6° at 40 pts)

            # Start fresh ranging -> wait for data -> read -> stop
            # This avoids waiting for a "stale" reading to complete
            self.vl53.start_ranging()
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
