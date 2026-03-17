from motor_driver import motor_driver
from encoder_driver import encoder
from task_share import Share, Queue
from utime import ticks_us, ticks_diff
import micropython


S0_INIT = micropython.const(0)  # init
S1_WAIT = micropython.const(1)  # wait for go
S2_RUN  = micropython.const(2)  # run control


class task_control:
    """
    Dual-mode control task.

    Mode A (backwards-compatible): Speed PI controller (one motor)
      - Provide: mot, enc, goFlag, effort_cmd, (optional) vel_setpoint Share
      - Used by your existing main.py

    Mode B (NEW): Line-following controller (differential drive)
      - Provide, in addition to the above:
          line_sensor=<L_sensor instance>   (must have calculate_error()->float)
          other_effort_cmd=<Share>          (effort command for the OTHER wheel)
      - Optional:
          base_effort_share=<Share> (float), else base_effort_local is used

      Behavior:
        error = line_sensor.calculate_error()
        steer = kp_line*error + kd_line*d(error)/dt
        left_cmd  = base + steer
        right_cmd = base - steer
    """

    def __init__(
        self,
        mot: motor_driver,
        enc: encoder,
        goFlag: Share,
        effort_cmd: Share,
        vel_setpoint=None,
        dataValues=None,
        timeValues=None,
        # --- Speed-setpoint line-follow additions (all optional) ---
        base_vel_share: Share = None,
        steer_gain_share: Share = None,
        line_follow_enable_share: Share = None,
        steer_sign: float = +1.0,
        # --- Effort-based line-follow mode (optional) ---
        line_sensor=None,
        other_effort_cmd: Share = None,
        other_goFlag: Share = None,
        base_effort_share: Share = None,
        right_trim_share: Share = None,   # multiplicative right-wheel trim (default 1.0)
        # --- IMU stabilization (optional) ---
        imu_heading_share: Share = None,
        imu_yawrate_share: Share = None,
        # --- Observer outputs (optional) ---
        psi_hat_share: Share = None,
        omL_hat_share: Share = None,
        omR_hat_share: Share = None,
        # Backwards-compatible alias name:
        line_error_share: Share = None,
        max_vel_share: Share = None,
    ):
        self._state = S0_INIT

        # Shares for speed-setpoint line-follow (optional)
        self._base_vel_share = base_vel_share
        self._steer_gain_share = steer_gain_share
        self._line_follow_enable_share = line_follow_enable_share
        self._line_error_share = line_error_share
        self._steer_sign = float(steer_sign)
        self._max_vel_share = max_vel_share

        # Local fallbacks if shares are not provided
        self._base_vel_local = 0.0
        self._steer_gain_local = 0.0
        self._mot = mot
        self._enc = enc
        self._goFlag = goFlag
        self._effort_cmd = effort_cmd
        self._vel_setpoint_share = vel_setpoint
        self._dataValues = dataValues
        self._timeValues = timeValues

        # Line-follow / dual-mode wiring
        self._line_sensor = line_sensor
        self._other_effort_cmd = other_effort_cmd
        self._other_goFlag = other_goFlag
        self._base_effort_share = base_effort_share
        self._right_trim_share  = right_trim_share
        self._right_offset      = 0.0   # additive PWM offset for right wheel trim

        # IMU stabilization (optional)
        self._imu_heading_share = imu_heading_share
        self._imu_yawrate_share = imu_yawrate_share
        self._k_yawrate = 0.0
        self._k_heading = 0.0
        self._heading_ref = None
        self._curve_threshold_dps = 5.0

        # Observer outputs (optional)
        self._psi_hat_share = psi_hat_share
        self._omL_hat_share = omL_hat_share
        self._omR_hat_share = omR_hat_share

        # Common timing/logging
        self._startTime = 0
        self._ramp_us = 500_000   # ramp duration in microseconds (0.5 s)
        self._last_t = 0

        # ---------------- Speed PI parameters ----------------
        self._kp = 0.08
        self._ki = 0.0
        self._vel_setpoint_local = 3000.0

        self._err_int = 0.0
        self._vel_filt = 0.0
        self._vel_filter_alpha = 0.75

        self._pwm_deadband = 5.0
        self._max_effort = 30.0   # hard cap on PWM output %
        self._pwm_deadband_hyst = 7.0

        # ---------------- Line-follow parameters ----------------
        self._kp_line = 0.0
        self._ki_line = 0.0    # integral gain — set via ctrl_obj._ki_line in main.py
        self._base_effort_local = 35.0  # % PWM (0..100); tune for your robot
        self._err_int_line = 0.0   # line error integrator

        # ---------------- Odometry ----------------
        # Dead-reckoning from encoder counts. Updated every control tick.
        # Reset to zero at the start of each run (when goFlag goes True).
        self._WHEEL_TRACK_MM = 141.0   # mm between wheel contact patches (Romi)
        self._MM_PER_COUNT   = 0.15303 # (2*pi*35) / 1437.07
        self._odo_x   = 0.0   # mm, global X (forward from start)
        self._odo_y   = 0.0   # mm, global Y (left from start)
        self._odo_h   = 0.0   # radians, heading (0 = straight ahead)
        self._odo_dist = 0.0  # mm, total straight-line distance from start
        self._odo_imu_h0 = None  # IMU heading (deg) at last odometry reset
        self._odo_h_base = 0.0   # heading (rad) set at last reset
        self._psi_hat_h0 = 0.0   # observer psi_hat (rad) at last odometry reset
        self._enc_l_prev = 0  # previous left encoder count
        self._enc_r_prev = 0  # previous right encoder count
        # reference to right encoder (must be set from main.py after construction)
        self._right_enc = None

        print("Control Task object instantiated")

    # --------- Helpers: decide which mode we're in ---------
    def _line_mode_enabled(self) -> bool:
        return (self._line_sensor is not None) and (self._other_effort_cmd is not None)

    def _odo_reset(self):
        """Reset odometry to zero at run start."""
        self._odo_x    = 0.0
        self._odo_y    = 0.0
        self._odo_h    = 0.0
        self._odo_h_base = 0.0
        self._odo_dist = 0.0
        self._psi_hat_h0 = 0.0
        if self._psi_hat_share is not None:
            try:
                p = self._psi_hat_share.get()
                if p is not None:
                    self._psi_hat_h0 = float(p)
            except Exception:
                pass
        self._odo_imu_h0 = None
        if self._imu_heading_share is not None:
            try:
                h0 = self._imu_heading_share.get()
                if h0 is not None:
                    self._odo_imu_h0 = float(h0)
            except Exception:
                pass
        self._enc_l_prev = self._enc.get_position()
        if self._right_enc is not None:
            self._enc_r_prev = self._right_enc.get_position()

    def _odo_update(self):
        """Update dead-reckoning position from encoder counts, with IMU-fused heading."""
        import math
        l_now = self._enc.get_position()
        r_now = self._right_enc.get_position() if self._right_enc is not None else self._enc_r_prev

        dl = -((l_now - self._enc_l_prev) * self._MM_PER_COUNT)
        dr = -((r_now - self._enc_r_prev) * self._MM_PER_COUNT)
        self._enc_l_prev = l_now
        self._enc_r_prev = r_now

        ds = (dl + dr) / 2.0

        if self._psi_hat_share is not None:
            try:
                psi = self._psi_hat_share.get()
                if psi is not None:
                    self._odo_h = self._odo_h_base + (float(psi) - self._psi_hat_h0)
            except Exception:
                self._odo_h += (dl - dr) / self._WHEEL_TRACK_MM
        elif self._odo_imu_h0 is not None and self._imu_heading_share is not None:
            try:
                h_now = self._imu_heading_share.get()
                if h_now is not None:
                    dh_deg = self._wrap_deg(float(h_now) - self._odo_imu_h0)
                    self._odo_h = self._odo_h_base + math.radians(dh_deg)
            except Exception:
                self._odo_h += (dl - dr) / self._WHEEL_TRACK_MM
        else:
            self._odo_h += (dl - dr) / self._WHEEL_TRACK_MM

        self._odo_x += ds * math.cos(self._odo_h)
        self._odo_y += ds * math.sin(self._odo_h)
        self._odo_dist = math.sqrt(self._odo_x ** 2 + self._odo_y ** 2)

    def get_odometry(self):
        """Return (x_mm, y_mm, heading_deg, straight_line_dist_mm)."""
        import math
        return (self._odo_x, self._odo_y,
                math.degrees(self._odo_h),
                self._odo_dist)

    def set_odometry(self, x_mm: float, y_mm: float, heading_deg: float) -> None:
        """
        Force odometry to a known pose.

        Useful when the robot hits a known landmark (e.g. wall at a known X)
        so accumulated drift can be reset.
        """
        import math
        self._odo_x = float(x_mm)
        self._odo_y = float(y_mm)
        self._odo_h = math.radians(float(heading_deg))
        self._odo_h_base = self._odo_h
        self._odo_dist = math.sqrt(self._odo_x ** 2 + self._odo_y ** 2)
        self._psi_hat_h0 = 0.0
        if self._psi_hat_share is not None:
            try:
                p = self._psi_hat_share.get()
                if p is not None:
                    self._psi_hat_h0 = float(p)
            except Exception:
                pass
        self._odo_imu_h0 = None
        if self._imu_heading_share is not None:
            try:
                h0 = self._imu_heading_share.get()
                if h0 is not None:
                    self._odo_imu_h0 = float(h0)
            except Exception:
                pass

    # ---------------- Speed-mode API ----------------
    def set_velocity_setpoint(self, vel_setpoint: float) -> None:
        self._vel_setpoint_local = float(vel_setpoint)

    # Backwards-compatible alias
    def set_set_point(self, setPoint) -> None:
        self.set_velocity_setpoint(setPoint)

    # FIX #1: _get_speed_setpoint was previously nested INSIDE set_set_point,
    # making it a local function that was never reachable as a method. It is
    # now correctly defined at class scope.
    def _get_speed_setpoint(self) -> float:
        # Compute the raw target setpoint (with steering correction if LF active)
        if self._line_follow_enable_share is not None and self._line_follow_enable_share.get():
            base = self._base_vel_local
            if self._base_vel_share is not None:
                v = self._base_vel_share.get()
                if v is not None:
                    base = float(v)

            k = self._steer_gain_local
            if self._steer_gain_share is not None:
                g = self._steer_gain_share.get()
                if g is not None:
                    k = float(g)

            e = 0.0
            if self._line_error_share is not None:
                err = self._line_error_share.get()
                if err is not None:
                    e = float(err)

            target = base + (self._steer_sign * k * e)
        else:
            if self._vel_setpoint_share is None:
                target = self._vel_setpoint_local
            else:
                sp = self._vel_setpoint_share.get()
                target = self._vel_setpoint_local if sp is None else float(sp)

        # Soft-start ramp: scale the setpoint linearly from 0 to target
        # over _ramp_us microseconds so the motor doesn't lurch at startup.
        elapsed = ticks_diff(ticks_us(), self._startTime)
        if elapsed < self._ramp_us:
            target = target * (elapsed / self._ramp_us)

        # Hard cap on maximum speed
        if self._max_vel_share is not None:
            max_v = self._max_vel_share.get()
            if max_v is not None and max_v > 0:
                if target > max_v:
                    target = max_v
                elif target < -max_v:
                    target = -max_v

        return target

    
    # --------- Helpers: angle math ---------
    @staticmethod
    def _wrap_deg(angle_deg: float) -> float:
        """Wrap angle to [-180, 180)."""
        a = angle_deg
        while a >= 180.0:
            a -= 360.0
        while a < -180.0:
            a += 360.0
        return a

    # ---------------- Line-mode API ----------------
    def set_base_effort(self, base_effort: float) -> None:
        self._base_effort_local = float(base_effort)

    def set_line_gains(self, kp: float) -> None:
        self._kp_line = float(kp)

    # IMU stabilization gains
    def set_imu_gains(self, yawrate_gain: float = 0.0, heading_gain: float = 0.0) -> None:
        """Set gains used in line-follow mode.
        yawrate_gain: multiplies gyro Z (deg/s) as a damping term.
        heading_gain: multiplies (heading - heading_ref) to reduce drift.
        """
        self._k_yawrate = float(yawrate_gain)
        self._k_heading = float(heading_gain)

    def _get_base_effort(self) -> float:
        if self._base_effort_share is None:
            return self._base_effort_local
        val = self._base_effort_share.get()
        return self._base_effort_local if val is None else float(val)

    def _get_right_trim(self) -> float:
        if self._right_trim_share is None:
            return 1.0
        val = self._right_trim_share.get()
        return 1.0 if val is None else float(val)

    # ---------------- Common stop ----------------
    def _log_pair_nonblocking(self, data_val, time_val):
        """Log one (data,time) sample to the optional queues without ever blocking.

        If either queue is full, we stop the run cleanly instead of blocking the scheduler.
        """
        if (self._dataValues is None) or (self._timeValues is None):
            return
        # Only log if BOTH queues have space so samples stay aligned
        if self._dataValues.full() or self._timeValues.full():
            self._stop_run()
            return
        self._dataValues.put(data_val)
        self._timeValues.put(time_val)

    def _stop_run(self):
        # ensure motor command(s) go to 0 and tell UI we're done
        self._effort_cmd.put(0.0)
        if self._line_mode_enabled():
            self._other_effort_cmd.put(0.0)
            if self._other_goFlag is not None:
                self._other_goFlag.put(False)
        self._goFlag.put(False)
        self._state = S1_WAIT

    # ---------------- Main generator ----------------
    def run(self):
        while True:

            if self._state == S0_INIT:
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                # Keep command(s) at 0 while waiting
                self._effort_cmd.put(0.0)
                if self._line_mode_enabled():
                    self._other_effort_cmd.put(0.0)

                if self._goFlag.get():
                    if self._line_mode_enabled() and (self._other_goFlag is not None):
                        self._other_goFlag.put(True)
                    self._startTime = ticks_us()
                    self._last_t = self._startTime

                    # Reset encoder timing so the first velocity sample after
                    # restart doesn't use a stale dt from when the motor was
                    # stopped, which would produce a huge velocity spike and
                    # saturate the PI output on the very first tick.
                    self._enc.zero()

                    # reset integrators/filters
                    self._err_int = 0.0
                    self._vel_filt = 0.0

                    # reset line controller memory

                    self._err_int_line = 0.0

                    # reset odometry to global zero
                    self._odo_reset()

                    # latch heading reference for heading-hold (prefer observer)
                    self._heading_ref = None
                    if self._line_mode_enabled():
                        if self._psi_hat_share is not None:
                            try:
                                p0 = self._psi_hat_share.get()
                                if p0 is not None:
                                    self._heading_ref = float(p0) * 57.2958
                            except Exception:
                                pass
                        if self._heading_ref is None and self._imu_heading_share is not None:
                            try:
                                h0 = self._imu_heading_share.get()
                                if h0 is not None:
                                    self._heading_ref = float(h0)
                            except Exception:
                                pass

                    self._state = S2_RUN

            elif self._state == S2_RUN:

                # If someone cleared the left goFlag (UI cancel), stop both motors
                if not self._goFlag.get():
                    self._effort_cmd.put(0.0)
                    if self._line_mode_enabled():
                        self._other_effort_cmd.put(0.0)
                        if self._other_goFlag is not None:
                            self._other_goFlag.put(False)
                    self._state = S1_WAIT
                    yield self._state
                    continue

                # If logging is enabled and buffers are full, stop BEFORE any put() can block
                if (self._dataValues is not None) and (self._timeValues is not None):
                    if self._dataValues.full() or self._timeValues.full():
                        self._stop_run()
                        yield self._state
                        continue

                if self._line_mode_enabled():
                    # ===================== LINE FOLLOW MODE =====================
                    now = ticks_us()
                    dt_us = ticks_diff(now, self._last_t)
                    self._last_t = now
                    if dt_us <= 0:
                        yield self._state
                        continue
                    dt_s = dt_us / 1_000_000.0

                    # Update both encoders so positions are fresh for odometry
                    self._enc.update()
                    if self._right_enc is not None:
                        self._right_enc.update()

                    err = float(self._line_sensor.calculate_error())

                    # Integrate error with anti-windup clamp
                    self._err_int_line += err * dt_s
                    self._err_int_line = max(-10.0, min(10.0, self._err_int_line))

                    base = self._get_base_effort()
                    steer = (self._kp_line * err) + (self._ki_line * self._err_int_line)

                    # ---- Yaw-rate damping (prefer observer, fallback to raw gyro) ----
                    if self._k_yawrate != 0.0:
                        try:
                            wz = None
                            if self._omL_hat_share is not None and self._omR_hat_share is not None:
                                oL = self._omL_hat_share.get()
                                oR = self._omR_hat_share.get()
                                if oL is not None and oR is not None:
                                    wz = (float(oR) - float(oL)) * 0.2482 * 57.2958
                            if wz is None and self._imu_yawrate_share is not None:
                                wz_raw = self._imu_yawrate_share.get()
                                if wz_raw is not None:
                                    wz = float(wz_raw)
                            if wz is not None:
                                steer -= (self._k_yawrate * wz)
                        except Exception:
                            pass

                    # Heading hold (degrees), suppressed on curves
                    if self._k_heading != 0.0 and (self._heading_ref is not None):
                        try:
                            h = None
                            wz_now = 0.0
                            if self._psi_hat_share is not None:
                                p = self._psi_hat_share.get()
                                if p is not None:
                                    h = float(p) * 57.2958
                            if h is None and self._imu_heading_share is not None:
                                h = self._imu_heading_share.get()
                                if h is not None:
                                    h = float(h)
                            if self._imu_yawrate_share is not None:
                                wz_raw = self._imu_yawrate_share.get()
                                if wz_raw is not None:
                                    wz_now = float(wz_raw)
                            if h is not None:
                                if abs(wz_now) > self._curve_threshold_dps:
                                    self._heading_ref = h
                                else:
                                    h_err = self._wrap_deg(h - float(self._heading_ref))
                                    steer -= (self._k_heading * h_err)
                        except Exception:
                            pass

                    left_out = base + steer
                    # RIGHT_OFFSET: positive value = right motor spins faster.
                    # task_motor inverts the command, so publishing a larger positive
                    # value results in a larger magnitude after negation -> more PWM.
                    right_out = (base - steer) + self._right_offset

                    # Coordinated scaling: if either wheel exceeds max_effort,
                    # scale BOTH down by the same factor so the differential
                    # (and therefore the turn) is always preserved.
                    peak = max(abs(left_out), abs(right_out))
                    if peak > self._max_effort:
                        scale = self._max_effort / peak
                        left_out  *= scale
                        right_out *= scale

                    # Publish effort commands for BOTH wheels
                    self._effort_cmd.put(left_out)
                    self._other_effort_cmd.put(right_out)

                    # Update dead-reckoning position
                    self._odo_update()

                    # Optional logging: store error vs time (non-blocking)
                    self._log_pair_nonblocking(err, ticks_diff(now, self._startTime))

                else:
                    # ===================== SPEED PI MODE (original) =====================
                    # Update encoder and compute velocity
                    self._enc.update()
                    vel_raw = self._enc.get_velocity()
                    dt = self._enc.dt
                    # Skip ticks where dt is zero or implausibly large (>500 ms).
                    # A large dt on the first tick after restart (even after enc.zero())
                    # can still produce a velocity spike that saturates the PI output.
                    if dt <= 0 or dt > 500_000:
                        yield self._state
                        continue

                    # Low-pass filter velocity
                    self._vel_filt = (self._vel_filter_alpha * vel_raw) + ((1.0 - self._vel_filter_alpha) * self._vel_filt)

                    # PI control
                    setpoint = self._get_speed_setpoint()
                    err = setpoint - self._vel_filt
                    self._err_int += err * dt
                    self._err_int = max(min(self._err_int, 200.0), -200.0)  # anti-windup clamp

                    effort = (self._kp * err) + (self._ki * self._err_int)

                    # Deadband + hysteresis, output limited to [-100, 100]
                    if abs(effort) > self._pwm_deadband:
                        out = effort
                    else:
                        try:
                            prev_pwm = abs(self._mot.pwm_ch.pulse_width_percent())
                        except Exception:
                            prev_pwm = 0.0
                        if prev_pwm > self._pwm_deadband_hyst:
                            out = self._pwm_deadband * (1 if effort >= 0 else -1)
                        else:
                            out = 0.0

                    if out > self._max_effort:
                        out = self._max_effort
                    elif out < -self._max_effort:
                        out = -self._max_effort

                    # Publish effort for motor_task to apply
                    self._effort_cmd.put(out)

                    # Optional logging (non-blocking)
                    t = ticks_us()
                    self._log_pair_nonblocking(self._vel_filt, ticks_diff(t, self._startTime))

            yield self._state
