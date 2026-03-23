Software Modules
================

The firmware is split into 18 Python modules. Each has a single responsibility.
Driver modules contain hardware-level logic; task modules contain the scheduler
logic (state machine + generator loop); utility modules are shared infrastructure.

.. raw:: html

   <div class="module-grid">

     <div class="module-card">
       <span class="mc-file">main.py</span>
       <span class="mc-desc">
         Entry point. Initialises all hardware, creates Share objects, instantiates
         task objects, registers them with the scheduler, then enters the infinite
         <code>pri_sched()</code> loop.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">cotask.py</span>
       <span class="mc-desc">
         Cooperative scheduler (JR Ridgely). Maintains a sorted priority list of
         generator tasks; each call to <code>pri_sched()</code> runs the highest-priority
         ready task for one generator step.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_share.py</span>
       <span class="mc-desc">
         Interrupt-safe inter-task communication. <code>Share</code> holds a single
         latest value; <code>Queue</code> buffers a sequence. Interrupts are disabled
         during read/write to prevent data corruption.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">motor_driver.py</span>
       <span class="mc-desc">
         DRV8833 H-bridge driver. Maps a signed effort value (−100 to +100 %) to
         direction pin + PWM duty cycle. Zero effort cleanly brakes the motor.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">encoder_driver.py</span>
       <span class="mc-desc">
         Quadrature encoder using STM32 timer encoder mode. Returns position in counts
         and velocity in counts/second (not counts/µs — a critical fix that prevented
         PI controller saturation).
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">imu_bno055.py</span>
       <span class="mc-desc">
         Minimal I²C driver for the BNO055. Configures IMUPLUS mode (accel + gyro
         only) for motor-field robustness. Reads Euler heading and gyro Z at 50 Hz.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">Line_sensor_driver.py</span>
       <span class="mc-desc">
         Reads the IR array and computes a weighted-centroid steering error.
         Supports per-sensor black/white calibration and dynamic bias adjustment.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">multi_sensor_read.py</span>
       <span class="mc-desc">
         Non-blocking ADC reader for the IR array. Replaced a blocking
         <code>ADC.read_timed_multi()</code> call that consumed 50 % of CPU every
         tick with instant individual <code>ADC.read()</code> calls.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">bump_sensor.py</span>
       <span class="mc-desc">
         Active-low GPIO driver for the two 3-button bumper boards. Returns boolean
         pressed/not-pressed states with convenience <code>any()</code>,
         <code>any_left()</code>, <code>any_right()</code> predicates.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_motor.py</span>
       <span class="mc-desc">
         Safety wrapper for each motor. 3-state machine: INIT → WAIT → RUN.
         Motors only run when both <code>armEnable</code> and <code>goFlag</code>
         are set. <code>_safe_off()</code> is called once on exit, not every tick.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_sensor.py</span>
       <span class="mc-desc">
         Periodic IR sensor task. Calls <code>calculate_error()</code> every tick
         and writes the result to <code>lineError</code> Share so both the control
         task and user task can read it.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_imu.py</span>
       <span class="mc-desc">
         Polls the BNO055 every 20 ms and publishes heading, yaw-rate, and packed
         calibration status. Automatically re-initialises the sensor if it goes
         offline.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">control_task.py</span>
       <span class="mc-desc">
         Dual-mode controller. In line-follow mode: PI steering + IMU yaw-rate
         damping + coordinated PWM scaling. In speed mode: single-motor PI
         velocity controller with soft-start ramp.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_state_estimator.py</span>
       <span class="mc-desc">
         Discretized Luenberger observer. Fuses encoder odometry, IMU heading, and
         motor voltage to estimate <em>s</em>, <em>ψ</em>, <em>ΩL</em>, <em>ΩR</em>
         at 50 Hz using ulab matrix multiply.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">task_user.py</span>
       <span class="mc-desc">
         14-state autonomous navigation FSM. Handles Bluetooth commands, line
         following, odometry-based forward drives, IMU heading turns, bump avoidance,
         and full obstacle course sequencing.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">compute_observer.py</span>
       <span class="mc-desc">
         PC-side design script (numpy + scipy). Designs the Luenberger gain via CARE,
         discretizes the closed-loop system matrices at Ts = 20 ms, and prints AD
         and BD for pasting into the estimator task.
       </span>
     </div>

     <div class="module-card">
       <span class="mc-file">bt_config.py</span>
       <span class="mc-desc">
         One-time HC-05 configuration utility. Sends AT commands over UART1 at
         38,400 baud (AT mode) to set device name, pairing password, and operating
         baud rate.
       </span>
     </div>

   </div>

.. toctree::
   :maxdepth: 1
   :caption: Module Details

   drivers
   tasks
   scheduler
