#!/usr/bin/env python3
"""Test script for MPU-6050/6500 IMU sensor."""

from sensors.imu import IMU

# Initialize IMU (calibrates gyro on startup - keep robot still!)
imu = IMU(calibrate=True)

print(f"Accel: {imu.read_accel()}")
print(f"Gyro: {imu.read_gyro()}")
print(f"Gyro Z: {imu.read_gyro_z():.2f} deg/s")
