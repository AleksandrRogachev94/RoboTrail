"""MPU-6050/6500 IMU sensor interface with gyro calibration.

Provides calibrated gyro readings for rotation measurement.
"""

import struct
import time

import board
import busio


class IMU:
    """MPU-6050/6500 IMU with automatic gyro bias calibration."""

    ADDR = 0x68

    def __init__(self, i2c=None, calibrate=True):
        """
        Initialize IMU.

        Args:
            i2c: Optional existing I2C bus. If None, creates one.
            calibrate: If True, runs gyro calibration on init (keep robot still!)
        """
        self.i2c = i2c or busio.I2C(board.SCL, board.SDA)
        self._owns_i2c = i2c is None

        # Gyro bias (found during calibration)
        self.gyro_bias = (0.0, 0.0, 0.0)

        # Wake up the sensor
        self._write_register(0x6B, 0x00)
        time.sleep(0.1)

        if calibrate:
            self.calibrate_gyro()

    def _write_register(self, register: int, value: int) -> None:
        """Write a single byte to a register."""
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.ADDR, bytes([register, value]))
        finally:
            self.i2c.unlock()

    def _read_raw(self, register: int, count: int) -> bytearray:
        """Read raw bytes from register."""
        while not self.i2c.try_lock():
            pass
        try:
            result = bytearray(count)
            self.i2c.writeto_then_readfrom(self.ADDR, bytes([register]), result)
            return result
        finally:
            self.i2c.unlock()

    def read_gyro_raw(self) -> tuple[float, float, float]:
        """Read raw gyro values in deg/s (without bias correction)."""
        raw = self._read_raw(0x43, 6)
        gx, gy, gz = struct.unpack(">hhh", raw)
        scale = 1.0 / 131.0  # ±250°/s range
        return (gx * scale, gy * scale, gz * scale)

    def read_gyro(self) -> tuple[float, float, float]:
        """Read calibrated gyro values in deg/s (with bias correction)."""
        gx, gy, gz = self.read_gyro_raw()
        return (
            gx - self.gyro_bias[0],
            gy - self.gyro_bias[1],
            gz - self.gyro_bias[2],
        )

    def read_gyro_z(self) -> float:
        """Read calibrated Z-axis angular velocity in deg/s."""
        return self.read_gyro()[2]

    def calibrate_gyro(self, samples: int = 200, delay: float = 0.005) -> None:
        """
        Calibrate gyro by measuring bias while stationary.

        Args:
            samples: Number of samples to average
            delay: Delay between samples in seconds

        IMPORTANT: Keep the robot completely still during calibration!
        """
        print("Calibrating gyro (keep robot still)...")

        sum_gx, sum_gy, sum_gz = 0.0, 0.0, 0.0

        for _ in range(samples):
            gx, gy, gz = self.read_gyro_raw()
            sum_gx += gx
            sum_gy += gy
            sum_gz += gz
            time.sleep(delay)

        self.gyro_bias = (
            sum_gx / samples,
            sum_gy / samples,
            sum_gz / samples,
        )
        print(f"Gyro bias: Z = {self.gyro_bias[2]:.3f} deg/s")

    def read_accel(self) -> tuple[float, float, float]:
        """Read accelerometer values in m/s²."""
        raw = self._read_raw(0x3B, 6)
        ax, ay, az = struct.unpack(">hhh", raw)
        scale = 9.80665 / 16384.0  # ±2g range
        return (ax * scale, ay * scale, az * scale)
