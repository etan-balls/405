Hardware Drivers
================

motor_driver.py
---------------

Wraps a single channel of the DRV8833 dual H-bridge. The driver exposes a clean
±100 % effort interface regardless of the underlying direction-pin + PWM wiring.

.. code-block:: python

   motor = motor_driver(Pin.cpu.B5,   # nSLP
                        Pin.cpu.B4,   # DIR
                        Pin.cpu.C8,   # PWM (TIM3 CH3)
                        tim3, 3)

   motor.enable()           # wake driver
   motor.set_effort(60)     # 60 % forward
   motor.set_effort(-40)    # 40 % reverse
   motor.set_effort(0)      # brake
   motor.disable()          # sleep (low-power)

**Key design decision:** ``set_effort`` uses a strict three-way split (``> 0``,
``< 0``, ``== 0``) so direction is never changed on a brake command. The previous
version fell through to the ``else`` branch on zero effort, latching an unintended
direction before the next non-zero command.

encoder_driver.py
-----------------

Uses STM32 timer hardware encoder mode — no CPU involvement in counting. Both channels
are configured as ``ENC_AB`` quadrature inputs.

.. code-block:: python

   enc = encoder(1, pyb.Pin.cpu.A8, pyb.Pin.cpu.A9)  # TIM1, left wheel

   enc.update()                # must call before reading velocity
   pos = enc.get_position()    # cumulative count (wraps handled)
   vel = enc.get_velocity()    # counts per second
   enc.zero()                  # reset position and timing

**Critical fix:** the original driver returned velocity in counts/µs, making typical
speeds ~0.002. Controllers expecting counts/s (e.g. setpoint = 3000) would saturate
immediately. The corrected formula is:

.. code-block:: python

   velocity = (delta / dt_us) * 1_000_000   # counts per second

imu_bno055.py
-------------

Minimal I²C driver for the BNO055. Deliberately avoids the full Adafruit library to
minimize heap usage on a MicroPython target.

.. code-block:: python

   imu = BNO055(i2c, address=0x28)
   imu.begin()                          # reset, configure IMUPLUS mode

   h, roll, pitch = imu.read_euler_deg()
   wx, wy, wz     = imu.read_gyro_dps()
   sys, gyr, acc, mag = imu.calib_status()   # 0..3 each

**IMUPLUS mode** fuses only accelerometer and gyroscope. Full NDOF (which adds the
magnetometer) is unreliable when motor currents are flowing because the motor fields
distort the compass reading.

Line_sensor_driver.py
---------------------

Computes a **weighted centroid** of normalized sensor readings to produce a signed
steering error. The error is zero when the line is centred, negative when the line
is to the left, positive when to the right.

Normalization per sensor:

.. math::

   \text{norm}_i = \text{clamp}\!\left(\frac{\text{ADC}_i - \text{white}_i}{\text{black}_i - \text{white}_i},\ 0,\ 1\right)

Weighted centroid:

.. math::

   \text{error} = \frac{\sum_i w_i \cdot \text{norm}_i}{\sum_i \text{norm}_i} + \text{bias}

Default weights for 7 sensors: ``[-3, -2, -1, 0, 1, 2, 3]``.

If the total normalized response is below 0.15 (all sensors near white — line lost),
the function returns 0.0 rather than a noisy centroid.

.. code-block:: python

   line = L_sensor(sensor_fun,
                   black=[1420,1372,1187,891,919,1213,569],
                   white=[256, 268, 260, 243,247, 256, 235],
                   bias=0.0,
                   sensor_count=7)

   err = line.calculate_error()     # float, centred at 0
   raw = line.get_raw_readings()    # list of 7 ADC ints (0–4095)

multi_sensor_read.py
--------------------

Provides a ``read()`` method that returns all ADC values instantly.

The original implementation used ``ADC.read_timed_multi()``, which blocks the CPU
for the entire timer period waiting for samples. At 100 Hz that was a **10 ms
blocking call** inside a 20 ms task — 50 % CPU stall that starved every other task.

The fix uses ``ADC.read()`` which returns the STM32 ADC's last background-converted
value instantly (the hardware runs in continuous mode):

.. code-block:: python

   readings = sensor_fun.read()   # [int, int, ...] — no blocking

bump_sensor.py
--------------

Active-low GPIO driver with internal pull-ups.

.. code-block:: python

   bumps = BumpSensors(
       left_pins  = [Pin.cpu.B12, Pin.cpu.B11, Pin.cpu.C7],
       right_pins = [Pin.cpu.A15, Pin.cpu.H0,  Pin.cpu.H1],
   )

   bumps.any()          # True if any of the 6 buttons is pressed
   bumps.any_left()     # True if any left button pressed
   bumps.read_left()    # [bool, bool, bool]
