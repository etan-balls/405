Hardware Reference
==================

The robot is built on a **Pololu Romi** chassis with a **STM32 Nucleo-L476RG**
(Shoe of Brian carrier board) as the main controller.

Microcontroller
---------------

.. list-table::
   :widths: 30 70
   :header-rows: 0

   * - **Board**
     - STM32 Nucleo-L476RG
   * - **CPU**
     - ARM Cortex-M4 @ 80 MHz
   * - **Flash**
     - 1 MB
   * - **RAM**
     - 128 KB
   * - **Firmware**
     - MicroPython (pyb HAL)
   * - **Carrier**
     - Shoe of Brian (ME405 lab board)

Motors & Motor Drivers
-----------------------

Two DRV8833 dual H-bridge chips drive the left and right gear motors.
PWM frequency is 20 kHz (Timer 3) to stay above the audible range.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Signal</th><th>Left Motor</th><th>Right Motor</th></tr>
     </thead>
     <tbody>
       <tr><td>nSLP (enable)</td><td>PB5</td><td>PB15</td></tr>
       <tr><td>DIR</td><td>PB4</td><td>PB14</td></tr>
       <tr><td>PWM (TIM3)</td><td>PC8 — CH3</td><td>PC9 — CH4</td></tr>
     </tbody>
   </table>

.. note::
   Both motor tasks use ``invert=True`` so that a positive effort command drives
   forward on both sides (the physical wiring is inverted for the right motor).

Quadrature Encoders
-------------------

Each wheel has a magnetic quadrature encoder read by an STM32 hardware timer
in encoder mode. The timer counts pulses without CPU intervention, so readings
are always current.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Encoder</th><th>Timer</th><th>Ch A</th><th>Ch B</th><th>Resolution</th></tr>
     </thead>
     <tbody>
       <tr><td>Left</td><td>TIM1</td><td>PA8</td><td>PA9</td><td>1437 counts/rev</td></tr>
       <tr><td>Right</td><td>TIM2</td><td>PA0</td><td>PA1</td><td>1437 counts/rev</td></tr>
     </tbody>
   </table>

Wheel radius = 35 mm → **0.153 mm per count** (``_M_PER_COUNT = 0.00015303``).

IR Reflectance Array
--------------------

A 7-element analog reflectance array (Pololu QTR-style) spans the front of the
chassis. Each sensor is read by a 12-bit ADC channel.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Sensor #</th><th>Position</th><th>Pin</th></tr>
     </thead>
     <tbody>
       <tr><td>1 (leftmost)</td><td>far left</td><td>PC2</td></tr>
       <tr><td>2</td><td>left-center</td><td>PC3</td></tr>
       <tr><td>3</td><td>left</td><td>PC4</td></tr>
       <tr><td>4 (center)</td><td>center</td><td>PC5</td></tr>
       <tr><td>5</td><td>right</td><td>PB0</td></tr>
       <tr><td>6</td><td>right-center</td><td>PB1</td></tr>
       <tr><td>7 (rightmost)</td><td>far right</td><td>PA4</td></tr>
     </tbody>
   </table>

**Calibration values** (update these after running calibration mode ``c``):

.. code-block:: python

   black_adc = [1420, 1372, 1187, 891, 919, 1213, 569]  # per-sensor, over black tape
   white_adc = [256,  268,  260,  243, 247,  256,  235]  # per-sensor, over white surface

IMU — BNO055
------------

The Bosch BNO055 9-DOF IMU is connected via I²C1 and configured in
**IMUPLUS mode** (accelerometer + gyroscope fusion only). Magnetometer fusion
is disabled because motor magnetic fields corrupt the compass reading.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Signal</th><th>Pin</th><th>Notes</th></tr>
     </thead>
     <tbody>
       <tr><td>I²C SDA</td><td>PB9</td><td>I²C1</td></tr>
       <tr><td>I²C SCL</td><td>PB8</td><td>I²C1, 400 kHz</td></tr>
       <tr><td>I²C Address</td><td>0x28</td><td>ADR pin = GND</td></tr>
       <tr><td>RST</td><td>—</td><td>Not wired (PC8 used by motor PWM)</td></tr>
     </tbody>
   </table>

The IMU provides:

- ``read_euler_deg()`` → heading, roll, pitch (1/16 °/LSB)
- ``read_gyro_dps()`` → ωx, ωy, ωz (1/16 °/s per LSB)

Bump Sensors
------------

Two 3-button bumper boards (left and right) give six active-low inputs.
STM32 internal pull-ups are enabled; pressing a bumper pulls the line to ground.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Board</th><th>Button 1</th><th>Button 2</th><th>Button 3</th></tr>
     </thead>
     <tbody>
       <tr><td>Left</td><td>PB12</td><td>PB11</td><td>PC7</td></tr>
       <tr><td>Right</td><td>PA15</td><td>PH0</td><td>PH1</td></tr>
     </tbody>
   </table>

HC-05 Bluetooth Module
----------------------

An HC-05 serial Bluetooth module provides a wireless terminal connection.
The MicroPython REPL is redirected to UART1 so all ``print()`` output and
``input()`` prompts go over Bluetooth.

.. raw:: html

   <table class="pin-table">
     <thead>
       <tr><th>Signal</th><th>Pin</th><th>Notes</th></tr>
     </thead>
     <tbody>
       <tr><td>UART TX (Nucleo)</td><td>PB6</td><td>UART1_TX → HC-05 RX</td></tr>
       <tr><td>UART RX (Nucleo)</td><td>PB7</td><td>UART1_RX ← HC-05 TX</td></tr>
       <tr><td>VCC</td><td>3.3 V or 5 V</td><td></td></tr>
       <tr><td>Operating baud</td><td>115,200</td><td>After bt_config.py is run</td></tr>
     </tbody>
   </table>

.. warning::
   The factory default baud rate is 9,600. Run ``bt_config.py`` **once** with the
   HC-05 in AT command mode to switch it to 115,200, then update ``main.py``
   accordingly.

User Button
-----------

The blue user button on the Nucleo (PC13, active-low) starts the obstacle course
without needing a Bluetooth terminal. The user task polls it every scheduler tick.
