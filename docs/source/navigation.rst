Autonomous Navigation
=====================

The user task (``task_user.py``) implements a **14-state finite state machine**
that drives the robot through a complete obstacle course without any human input.
Start the course by pressing the blue Nucleo user button or sending ``o`` over
Bluetooth.

Bluetooth Commands
------------------

From a PuTTY terminal connected to the HC-05:

.. list-table::
   :widths: 15 85
   :header-rows: 1

   * - Key
     - Action
   * - ``o`` / ``O``
     - Start the full autonomous obstacle course (S4)
   * - ``f`` / ``F``
     - Start a timed line-following tuning run — prints error + odometry live (S2)
   * - ``c`` / ``C``
     - Enter calibration mode — prints raw IR sensor values (S3)
   * - any other key
     - Stop all motors immediately

State Machine
-------------

.. raw:: html

   <div class="state-flow">
     <span class="state-box">S0<br>INIT</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S1<br>IDLE</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S4<br>LINE FOLLOW 1</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S5<br>FWD DRIVE</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S7<br>→ WALL</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S9<br>BACK UP</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S6<br>TURN</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S10<br>FIND LINE</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S12<br>LINE FOLLOW 3</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S13<br>LINE FOLLOW 4</span>
     <span class="state-arrow">→</span>
     <span class="state-box">S1<br>DONE</span>
   </div>

State Descriptions
------------------

**S0 — INIT**
   Disables all motors and outputs ``READY`` to the terminal, then transitions
   immediately to S1.

**S1 — IDLE**
   Waits for a Bluetooth command or a button press. Polls both the serial input
   and the user button (PC13) every scheduler tick.

**S4 — LINE FOLLOW 1**
   Enables the line-following controller and follows the line. Transitions to S5
   when the line error has been zero for 3 consecutive ticks *and* the robot has
   travelled at least ``lf_min_dist`` mm — indicating the end of a line segment.
   Logs ``error, x, y, heading`` to Bluetooth every 20 ms.

**S5 — FORWARD DRIVE**
   Drives directly forward a fixed distance (default 50 mm) using encoder dead-
   reckoning. Uses a small heading correction from the odometry to drive straight.
   Transitions to S6 (turn) when the target distance is reached.

**S6 — TURN**
   Executes a heading-controlled turn to a target angle using IMU odometry.
   Supports both arc turns (one wheel faster, both forward) and spin turns (wheels
   counter-rotating). The proportional turn effort is:

   .. code-block:: python

      effort = min(20.0, max(10.0, remaining_degrees * 0.10))

   Transitions to the next state (``otnxt``) when the heading error is within 2°.

**S7 — DRIVE TO WALL**
   Drives forward at fixed effort (30 %) toward the wall, applying a heading
   correction to stay aimed at 170°. Polls bump sensors every tick. Transitions to
   S9 when any bumper fires.

**S9 — BACK UP**
   Reverses at fixed effort until the robot has moved 10 mm from the impact point.
   Then transitions to S6 to turn 90° (target 90°) toward the line.

**S10 — FIND LINE**
   Drives forward slowly while polling ``calculate_error()``. Transitions to S4 as
   soon as the line error becomes non-zero (line detected), passing odometry through
   to S4 so position is not lost.

**S12 — LINE FOLLOW 3**
   Second full line-follow segment. Transitions to S5 when the line error is zero
   for 3 ticks and heading is within ±30° of forward (0°).

**S13 — LINE FOLLOW 4 (biased)**
   Final line-follow segment with a small positive bias applied to the error so the
   robot deliberately drifts toward the right side of the tape — used to align with
   a finishing gate. Transitions to S6 (final turn to 270°) when done.

Odometry
--------

Dead-reckoning position is maintained inside ``task_control`` using encoder deltas
and the observer heading estimate:

.. raw:: html

   <div class="math-block">ds = (dL + dR) / 2       (average displacement in mm)

x += ds · cos(ψ)
y += ds · sin(ψ)
straight_line_dist = sqrt(x² + y²)</div>

The odometry is reset to ``(sx, sy, 0)`` at the start of the obstacle course run.
The coordinate system has +X pointing forward and +Y pointing left.

The user task calls ``set_odometry(x, y, heading)`` at known landmarks (e.g. after
completing a turn to a known heading) to correct accumulated drift.

Cancelling
----------

Sending any Bluetooth character during a run prints the final odometry and returns
to the IDLE state:

.. code-block:: text

   X=123.4 Y=456.7 H=270.0 D=512.3
   CANCEL
   >:
