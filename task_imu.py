# task_imu.py
# IMU task for ME405 Romi (BNO055).
#
# Shares produced:
#   - imu_heading_deg (float)
#   - imu_yaw_rate_dps (float)
#   - imu_calib (byte) packed SYS|GYR|ACC|MAG: (sys<<6)|(gyr<<4)|(acc<<2)|mag
#
# Nucleo-L476RG pin mapping:
#   PB8  -> I2C1_SCL
#   PB9  -> I2C1_SDA
#   PC8  -> BNO055 RST

import pyb
from imu_bno055 import BNO055


class task_imu:
    def __init__(self, i2c, heading_share, yawrate_share, calib_share,
                 address=0x28, use_ext_crystal=True, rst_pin=None):
        self.i2c = i2c
        self.heading = heading_share
        self.yawrate = yawrate_share
        self.calib = calib_share

        self.imu = BNO055(i2c, address=address, rst_pin=rst_pin)
        self._ok = self.imu.begin(use_ext_crystal=use_ext_crystal, do_hw_reset=True)

        self._last = pyb.millis()
        self._period_ms = 20  # 50 Hz

    def run(self):
        while True:
            if not self._ok:
                if pyb.elapsed_millis(self._last) > 1000:
                    self._last = pyb.millis()
                    self._ok = self.imu.begin()
                yield 0
                continue

            if pyb.elapsed_millis(self._last) >= self._period_ms:
                self._last = pyb.millis()
                try:
                    h, _, _ = self.imu.read_euler_deg()
                    _, _, gz = self.imu.read_gyro_dps()
                    sys, gyr, acc, mag = self.imu.calib_status()
                    packed = (sys << 6) | (gyr << 4) | (acc << 2) | mag

                    self.heading.put(h)
                    self.yawrate.put(gz)
                    self.calib.put(packed)
                except Exception:
                    self._ok = False

            yield 0
