Control Strategy
================

The Romi uses a **PI line-following controller** augmented with
IMU-based yaw-rate damping and coordinated motor scaling.

Line Error
----------

The sensor task computes a signed steering error every 20 ms using the weighted
centroid of the 7 IR sensors (see :doc:`software/drivers` for the math).
The error is:

- **0** when the line is centred under the array
- **negative** when the line is to the left
- **positive** when the line is to the right

PI Controller
-------------

.. raw:: html

   <div class="math-block">integral += err · dt       (anti-windup: clamped to ±10)

steer = Kp · err  +  Ki · integral</div>

Start with ``Ki = 0`` and tune ``Kp`` until the robot tracks well, then slowly
add ``Ki`` to correct steady-state drift on long straight sections.

IMU Yaw-Rate Damping
--------------------

Raw PI steering can cause the robot to oscillate — it overshoots a correction, then
corrects back, creating a sinusoidal weave. The gyro Z reading provides a direct
measure of how fast the robot is rotating, which can cancel this oscillation:

.. raw:: html

   <div class="math-block">wz = (ΩR_hat − ΩL_hat) · (r/w) · (180/π)   [deg/s, from observer]

steer = steer − K_yawrate · wz</div>

When the robot is straight (``wz ≈ 0``) this term has no effect. When the robot is
over-rotating during a correction, it subtracts from the steer command to damp it.

Typical value: ``K_yawrate = 0.030``. Increase if the robot still oscillates;
decrease if it feels sluggish on curves.

Differential Drive Output
--------------------------

.. raw:: html

   <div class="math-block">left_cmd  = base_effort + steer
right_cmd = base_effort − steer + right_offset</div>

``right_offset`` is a fixed additive trim (typically 0–4 %) to correct mechanical
asymmetry. If the robot drifts left on a straight, increase this value.

Coordinated Scaling
-------------------

If either wheel command exceeds ``max_effort``, both commands are scaled down by the
same factor so the **turn differential is always preserved**:

.. raw:: html

   <div class="math-block">peak = max(|left_cmd|, |right_cmd|)
if peak > max_effort:
    scale = max_effort / peak
    left_cmd  *= scale
    right_cmd *= scale</div>

This prevents the common problem where a sharp turn near full speed causes one wheel
to clip at 100 % while the other keeps going, effectively straightening the robot.

Tuning Guide
------------

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Parameter
     - File location
     - Effect
   * - ``BASE_EFFORT``
     - ``main.py``
     - Forward speed (%). Start at 20–25; raise when gains are tuned.
   * - ``KP_LINE``
     - ``main.py``
     - Proportional gain. Raise until tracking is sharp; back off if oscillating.
   * - ``KI_LINE``
     - ``main.py``
     - Integral gain. Start at 0; add slowly (0.5–2) for straight-line correction.
   * - ``MAX_EFFORT``
     - ``main.py``
     - PWM ceiling. Keep at 90–100 % for best performance.
   * - ``RIGHT_OFFSET``
     - ``main.py``
     - Right motor trim. Tune on a straight line; encoder diff should stay near 0.
   * - ``YAW_RATE_GAIN``
     - ``main.py``
     - Gyro damping. Raise from 0 in steps of 0.01 until oscillation disappears.

Calibration Mode
----------------

Send ``c`` over Bluetooth to enter calibration mode. The sensor task prints all
9 raw ADC values every 200 ms. Then:

- Press ``w`` with the sensors over **white** → records white calibration
- Press ``b`` with the sensors over **black** → records black calibration
- Press ``x`` to exit

Update ``black_adc`` and ``white_adc`` in ``main.py`` with the values you measured.
