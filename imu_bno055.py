# imu_bno055.py
# Minimal BNO055 driver for MicroPython (pyb.I2C) with optional hardware reset pin.
#
# Provides:
#   - begin() : optional HW reset, set to NDOF mode
#   - read_euler_deg() -> (heading, roll, pitch) in degrees
#   - read_gyro_dps()  -> (x, y, z) in deg/s
#   - calib_status()   -> (sys, gyro, accel, mag) each 0..3
#
# Notes:
# - Default I2C address is 0x28 (ADR pin low). Some boards use 0x29.
# - Euler scale: 1/16 deg per LSB. Gyro scale: 1/16 dps per LSB.
# - If you wire BNO055 RST to a MCU pin (PC8), pass rst_pin=pyb.Pin(...).

import pyb
import time


class BNO055:
    # I2C addresses
    ADDR_A = 0x28
    ADDR_B = 0x29

    # Registers
    _CHIP_ID = 0x00
    _OPR_MODE = 0x3D
    _PWR_MODE = 0x3E
    _SYS_TRIGGER = 0x3F
    _UNIT_SEL = 0x3B
    _CALIB_STAT = 0x35

    # Euler data (1/16 deg per LSB)
    _EULER_H_LSB = 0x1A  # heading
    _EULER_R_LSB = 0x1C  # roll
    _EULER_P_LSB = 0x1E  # pitch

    # Gyro data (1/16 dps per LSB)
    _GYR_X_LSB = 0x14
    _GYR_Y_LSB = 0x16
    _GYR_Z_LSB = 0x18

    # Modes
    _MODE_CONFIG   = 0x00
    _MODE_IMUPLUS  = 0x08   # accel + gyro only — stable near motors
    _MODE_NDOF     = 0x0C   # accel + gyro + mag — affected by motor fields

    # Power modes
    _PWR_NORMAL = 0x00

    def __init__(self, i2c: pyb.I2C, address: int = ADDR_A, rst_pin=None):
        self.i2c = i2c
        self.addr = address
        self.rst = rst_pin

    def hw_reset(self):
        """Pulse reset pin low then high (if provided)."""
        if self.rst is None:
            return
        try:
            self.rst.value(0)
            time.sleep_ms(10)
            self.rst.value(1)
            time.sleep_ms(700)
        except Exception:
            pass

    def _w8(self, reg: int, val: int):
        self.i2c.mem_write(bytes([val & 0xFF]), self.addr, reg)

    def _r(self, reg: int, n: int) -> bytes:
        return self.i2c.mem_read(n, self.addr, reg)

    @staticmethod
    def _s16(lo: int, hi: int) -> int:
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v

    def begin(self, use_ext_crystal: bool = True, do_hw_reset: bool = True) -> bool:
        if do_hw_reset:
            self.hw_reset()

        # Check chip ID (0xA0)
        try:
            chip = int.from_bytes(self._r(self._CHIP_ID, 1), "little")
        except Exception:
            return False

        if chip != 0xA0:
            time.sleep_ms(650)
            try:
                chip = int.from_bytes(self._r(self._CHIP_ID, 1), "little")
            except Exception:
                return False
            if chip != 0xA0:
                return False

        # CONFIG mode
        self.set_mode(self._MODE_CONFIG)
        time.sleep_ms(25)

        # Normal power
        self._w8(self._PWR_MODE, self._PWR_NORMAL)
        time.sleep_ms(10)

        # default units: degrees, dps
        self._w8(self._UNIT_SEL, 0x00)
        time.sleep_ms(10)

        if use_ext_crystal:
            self._w8(self._SYS_TRIGGER, 0x80)
            time.sleep_ms(10)

        self.set_mode(self._MODE_IMUPLUS)
        time.sleep_ms(25)
        return True

    def set_mode(self, mode: int):
        self._w8(self._OPR_MODE, mode & 0xFF)
        time.sleep_ms(10)

    def calib_status(self):
        b = int.from_bytes(self._r(self._CALIB_STAT, 1), "little")
        sys = (b >> 6) & 0x03
        gyr = (b >> 4) & 0x03
        acc = (b >> 2) & 0x03
        mag = (b >> 0) & 0x03
        return (sys, gyr, acc, mag)

    def read_euler_deg(self):
        data = self._r(self._EULER_H_LSB, 6)
        h = self._s16(data[0], data[1]) / 16.0
        r = self._s16(data[2], data[3]) / 16.0
        p = self._s16(data[4], data[5]) / 16.0
        return (h, r, p)

    def read_gyro_dps(self):
        data = self._r(self._GYR_X_LSB, 6)
        x = self._s16(data[0], data[1]) / 16.0
        y = self._s16(data[2], data[3]) / 16.0
        z = self._s16(data[4], data[5]) / 16.0
        return (x, y, z)
