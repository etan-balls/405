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
        self._kp_line = 22.0
        self._kd_line = 2.0
        self._base_effort_local = 35.0  # % PWM (0..100); tune for your robot
        self._err_prev = 0.0

        print("Control Task object instantiated")

    # --------- Helpers: decide which mode we're in ---------
    def _line_mode_enabled(self) -> bool:
        return (self._line_sensor is not None) and (self._other_effort_cmd is not None)

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

    # ---------------- Line-mode API ----------------
    def set_base_effort(self, base_effort: float) -> None:
        self._base_effort_local = float(base_effort)

    def set_line_gains(self, kp: float, kd: float = 0.0) -> None:
        self._kp_line = float(kp)
        self._kd_line = float(kd)

    def _get_base_effort(self) -> float:
        if self._base_effort_share is None:
            return self._base_effort_local
        val = self._base_effort_share.get()
        return self._base_effort_local if val is None else float(val)

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
                    self._err_prev = 0.0

                    self._state = S2_RUN

            elif self._state == S2_RUN:

                # If someone cleared the flag (UI 'q', etc.), stop immediately
                if (not self._goFlag.get()) or (self._line_mode_enabled() and (self._other_goFlag is not None) and (not self._other_goFlag.get())):
                    self._effort_cmd.put(0.0)
                    if self._line_mode_enabled():
                        self._other_effort_cmd.put(0.0)
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

                    err = float(self._line_sensor.calculate_error())
                    derr = (err - self._err_prev) / dt_s
                    self._err_prev = err

                    base = self._get_base_effort()
                    steer = (self._kp_line * err) + (self._kd_line * derr)

                    left_out = base + steer
                    right_out = base - steer

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
