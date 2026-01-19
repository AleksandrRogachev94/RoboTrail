import board
import busio
import struct
import time
class MPU6500:
    """Direct register access for MPU6050/6500/9250 family."""
    
    ADDR = 0x68
    
    def __init__(self, i2c):
        self.i2c = i2c
        while not self.i2c.try_lock():
            pass
        
        # Wake up (clear sleep bit in PWR_MGMT_1)
        self.i2c.writeto(self.ADDR, bytes([0x6B, 0x00]))
        time.sleep(0.1)
        self.i2c.unlock()
    
    def _read_raw(self, register, count):
        """Read raw bytes from register."""
        while not self.i2c.try_lock():
            pass
        result = bytearray(count)
        self.i2c.writeto_then_readfrom(self.ADDR, bytes([register]), result)
        self.i2c.unlock()
        return result
    
    @property
    def acceleration(self):
        """Returns (ax, ay, az) in m/s^2."""
        raw = self._read_raw(0x3B, 6)
        # Big-endian signed 16-bit, scale ±2g = 16384 LSB/g
        ax, ay, az = struct.unpack(">hhh", raw)
        scale = 9.80665 / 16384.0
        return (ax * scale, ay * scale, az * scale)
    
    @property
    def gyro(self):
        """Returns (gx, gy, gz) in deg/s."""
        raw = self._read_raw(0x43, 6)
        # Big-endian signed 16-bit, scale ±250°/s = 131 LSB/°/s
        gx, gy, gz = struct.unpack(">hhh", raw)
        scale = 1.0 / 131.0
        return (gx * scale, gy * scale, gz * scale)
# Usage
i2c = busio.I2C(board.SCL, board.SDA)
mpu = MPU6500(i2c)
print(f"Accel: {mpu.acceleration}")
print(f"Gyro: {mpu.gyro}")