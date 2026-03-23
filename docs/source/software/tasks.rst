Task Descriptions
=================

Every task is a Python class whose ``run()`` method is a **generator** (it contains
``yield`` statements). The cooperative scheduler calls ``next()`` on the generator
each time the task's period fires. The generator runs until it hits ``yield``, at
which point it returns the current state integer and suspends.

task_motor.py
-------------

A 3-state safety wrapper around each motor driver.

.. code-block:: text

   S0_INIT  →  S1_WAIT  →  S2_RUN
                 ↑              |
                 └──────────────┘
                 (armEnable or goFlag cleared)

- **S0_INIT**: calls ``_safe_off()`` once at startup to ensure the motor is off,
  then transitions to S1_WAIT.
- **S1_WAIT**: does *nothing* every tick — the hardware was already shut down on
  entry. Transitions to S2_RUN only when both ``armEnable`` and ``goFlag`` are
  simultaneously True.
- **S2_RUN**: reads ``effort_cmd`` Share and calls ``motor.set_effort()``. If
  either flag goes False, calls ``_safe_off()`` once and returns to S1_WAIT.

.. important::
   ``_safe_off()`` is called **once on exit** from S2_RUN and **once on entry** at
   init — not every tick in S1_WAIT. The original implementation called it every
   tick, which re-disabled the hardware immediately after the user task set the
   flags, making the motor impossible to restart.

task_sensor.py
--------------

Reads the IR array and publishes the error every tick.

.. code-block:: text

   S0_INIT  →  S1_RUN (forever)

Always publishes, even when line following is disabled, so the user task can
display live sensor data during tuning.

task_imu.py
-----------

Polls the BNO055 at 50 Hz and publishes to three Shares:

- ``imu_heading_deg`` — absolute Euler heading (°)
- ``imu_yaw_rate_dps`` — gyro Z (°/s)
- ``imu_calib`` — packed calibration byte: ``(sys<<6)|(gyr<<4)|(acc<<2)|mag``

If the BNO055 fails to initialise or raises an exception, the task waits 1 second
and retries ``imu.begin()``.

control_task.py
---------------

The master controller task that drives both wheels. It operates in two modes
depending on how it was constructed:

**Line-Follow Mode** (used in this project):

.. code-block:: python

   ctrl = task_control(
       mot=leftMotor, enc=leftEncoder,
       goFlag=leftMotorGo, effort_cmd=leftEffortCmd,
       line_sensor=line,                    # enables line-follow mode
       other_effort_cmd=rightEffortCmd,
       other_goFlag=rightMotorGo,
       base_effort_share=baseEffort,
       imu_heading_share=imu_heading_deg,
       imu_yawrate_share=imu_yaw_rate_dps,
   )

In line-follow mode, each tick:

1. Reads line error from ``line_sensor.calculate_error()``
2. Accumulates PI integral (anti-windup clamped to ±10)
3. Computes steering: ``steer = kp * err + ki * integral``
4. Applies yaw-rate damping: ``steer -= k_yawrate * wz``
5. Computes wheel commands: ``left = base + steer``, ``right = base − steer + offset``
6. Applies coordinated scaling so the turn differential is preserved at the PWM ceiling
7. Publishes both effort commands and updates dead-reckoning odometry

**Tunable parameters** (set on the ``ctrl_obj`` instance in ``main.py``):

.. list-table::
   :widths: 25 15 60
   :header-rows: 1

   * - Parameter
     - Default
     - Effect
   * - ``_kp_line``
     - 8.5
     - Proportional gain. Increase for sharper cornering; too high causes oscillation.
   * - ``_ki_line``
     - 2.0
     - Integral gain. Corrects steady-state offset on long straights. Start at 0.
   * - ``_base_effort_local``
     - 25 %
     - Forward speed. Raise for faster runs; lower reduces corner skidding.
   * - ``_max_effort``
     - 100 %
     - Hard PWM ceiling. Must be > ``base_effort`` or steering has no headroom.
   * - ``_right_offset``
     - 0.0
     - Additive % to right wheel for straight-line trim. Increase if robot curves left.
   * - ``_k_yawrate``
     - 0.030
     - Gyro damping gain. Increase if robot oscillates on straights.

task_state_estimator.py
-----------------------

Runs one step of the discretized Luenberger observer per tick. See the
:doc:`../observer` page for full mathematical details.

task_user.py
------------

The highest-level task. It handles all user interaction and autonomous navigation.
See the :doc:`../navigation` page for the full state machine description.
