System Architecture
===================

The Romi firmware is organized as a set of **cooperative generator tasks** managed by a
priority-based scheduler (``cotask.py``). There is no RTOS — each task is a Python
generator that executes for a bounded slice of time, then ``yield``\s control back to
the scheduler.

All inter-task communication goes through **interrupt-safe shared variables** provided
by ``task_share.py``. No task ever directly reads another task's internal state.

Scheduler
---------

The scheduler lives in ``cotask.py`` and is originally authored by JR Ridgely (Cal Poly).
It maintains a **sorted priority list**; each call to ``pri_sched()`` finds the
highest-priority task that is *ready* (past its period deadline) and runs it for exactly
one generator step.

All seven tasks are registered with the same 20 ms period (50 Hz), so the scheduler
runs on a tight 20 ms heartbeat:

.. raw:: html

   <table class="priority-table">
     <thead>
       <tr>
         <th>Priority</th>
         <th>Task Name</th>
         <th>Module</th>
         <th>Period</th>
         <th>Role</th>
       </tr>
     </thead>
     <tbody>
       <tr>
         <td><span class="pri-badge pri-5">5</span></td>
         <td><code>Sensor</code></td>
         <td><code>task_sensor.py</code></td>
         <td>20 ms</td>
         <td>Read 7 IR sensors, publish line error</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-5">5</span></td>
         <td><code>IMU</code></td>
         <td><code>task_imu.py</code></td>
         <td>20 ms</td>
         <td>Poll BNO055, publish heading &amp; yaw rate</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-4">4</span></td>
         <td><code>Ctrl</code></td>
         <td><code>control_task.py</code></td>
         <td>20 ms</td>
         <td>PI + line-follow controller, both wheels</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-3">3</span></td>
         <td><code>MotL</code></td>
         <td><code>task_motor.py</code></td>
         <td>20 ms</td>
         <td>Apply effort to left motor via H-bridge</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-3">3</span></td>
         <td><code>MotR</code></td>
         <td><code>task_motor.py</code></td>
         <td>20 ms</td>
         <td>Apply effort to right motor via H-bridge</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-2">2</span></td>
         <td><code>Est</code></td>
         <td><code>task_state_estimator.py</code></td>
         <td>20 ms</td>
         <td>Luenberger observer: x̂ = AD·x̂ + BD·u*</td>
       </tr>
       <tr>
         <td><span class="pri-badge pri-1">1</span></td>
         <td><code>User</code></td>
         <td><code>task_user.py</code></td>
         <td>20 ms</td>
         <td>Bluetooth REPL + full autonomous FSM</td>
       </tr>
     </tbody>
   </table>

Shared Variables
----------------

Tasks exchange data through ``Share`` objects — interrupt-protected single-value
buffers. Key shares:

.. list-table::
   :widths: 25 12 63
   :header-rows: 1

   * - Share name
     - Type
     - Description
   * - ``lineError``
     - float
     - Weighted-centroid line error from IR array (written by Sensor, read by Ctrl)
   * - ``leftEffortCmd``
     - float
     - Left motor PWM % (written by Ctrl, read by MotL and Est)
   * - ``rightEffortCmd``
     - float
     - Right motor PWM % (written by Ctrl, read by MotR and Est)
   * - ``imu_heading_deg``
     - float
     - Absolute Euler heading in degrees (written by IMU, read by Ctrl and Est)
   * - ``imu_yaw_rate_dps``
     - float
     - Gyro Z in deg/s (written by IMU, read by Ctrl and Est)
   * - ``psi_hat``
     - float
     - Observer heading estimate in radians (written by Est, read by Ctrl and User)
   * - ``s_hat``
     - float
     - Observer arc-length estimate in metres (written by Est)
   * - ``omL_hat`` / ``omR_hat``
     - float
     - Observer left/right wheel speed in rad/s (written by Est, read by Ctrl)
   * - ``armEnable``
     - bool
     - Safety interlock — motors only run when this is True
   * - ``lineFollowEnable``
     - bool
     - Enables line following in Ctrl; when False, Ctrl outputs zero effort

Data Flow Diagram
-----------------

.. code-block:: text

    ┌─────────────┐   line_err    ┌──────────────┐   effort L/R  ┌──────────┐
    │   Sensor    │──────────────▶│    Control   │──────────────▶│  MotorL  │
    │  (IR array) │               │    Task      │               │  MotorR  │
    └─────────────┘               │  (PI + steer)│               └──────────┘
                                  └──────┬───────┘
    ┌─────────────┐   ψ, ψ̇             │ omL_hat, omR_hat
    │     IMU     │─────────────────────┤
    │  (BNO055)   │            ┌────────▼────────┐
    └──────┬──────┘            │  State Est.     │
           │  ψ, ψ̇             │  (Luenberger)   │
           └───────────────────▶  x̂={s,ψ,ΩL,ΩR} │
                                └────────────────┘

    ┌─────────────┐   counts
    │  Encoders   │──────────────▶ State Est. + Odometry (inside Ctrl)
    └─────────────┘

    ┌─────────────┐
    │  Bump Sens. │──── fires ──▶ User Task (obstacle avoidance FSM)
    └─────────────┘
