# task_user.py - f=line follow (10s), x=cancel, c=calibrate, l/r=motor test (2s)

import sys
import uselect
import micropython
from utime import ticks_ms, ticks_diff
from task_share import Share

S0_INIT    = micropython.const(0)
S1_CMD     = micropython.const(1)
S4_LF      = micropython.const(4)
S5_CAL     = micropython.const(5)   # calibration mode
S6_MOT_L   = micropython.const(6)   # left motor test (2 s)
S7_MOT_R   = micropython.const(7)   # right motor test (2 s)
S8_STRAIGHT  = micropython.const(8)  # straight-line trim test (3 s)

LF_DURATION_MS      = micropython.const(10_000)  # 10 seconds
MOT_TEST_MS         = micropython.const(2_000)   # 2 seconds
STRAIGHT_TEST_MS    = micropython.const(3_000)   # 3 seconds
MOT_TEST_EFFORT     = micropython.const(30)      # PWM % for motor test
CAL_PRINT_MS        = micropython.const(200)     # print interval during cal
MM_PER_COUNT        = 0.15303                    # wheel radius 35mm, 1437.07 CPR

UI_prompt = ">: "
N_SENSORS = 9


class _IO:
    def __init__(self):
        self._poll = uselect.poll()
        self._poll.register(sys.stdin, uselect.POLLIN)

    def any(self):
        try:
            return 1 if self._poll.poll(0) else 0
        except Exception:
            return 0

    def read1(self):
        if not self.any():
            return None
        try:
            ch = sys.stdin.read(1)
            if not ch:
                return None
            return ch.encode() if isinstance(ch, str) else ch
        except Exception:
            return None

    def write(self, s):
        try:
            sys.stdout.write(s)
        except Exception:
            pass


class task_user:
    def __init__(self,
                 leftMotorGo: Share, rightMotorGo: Share,
                 lineFollowEnable: Share,
                 armEnable: Share,
                 lineSensor,
                 lineError: Share = None,
                 leftEffortCmd: Share = None,    # needed for motor test
                 rightEffortCmd: Share = None,   # needed for motor test
                 leftEncoder=None,               # encoder for position printing
                 rightEncoder=None,              # encoder for position printing
                 leftMotorDriver=None,           # motor driver object for direct test control
                 rightMotorDriver=None,          # motor driver object for direct test control
                 imu_heading: Share = None,      # BNO055 Euler heading (deg)
                 imu_yawrate: Share = None,      # BNO055 gyro Z (deg/s)
                 imu_calib: Share = None,        # packed calibration status byte
                 right_offset: float = 0.0,     # additive right-wheel trim for straight test
                 ctrl=None,                     # control task object for odometry readout
                 s_hat_share: Share = None,      # state estimator arc-length (m)
                 psi_hat_share: Share = None,   # state estimator heading (rad)
                 omL_hat_share: Share = None,   # state estimator left wheel speed (rad/s)
                 omR_hat_share: Share = None):  # state estimator right wheel speed (rad/s)
        self._state        = S0_INIT
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._lf_en        = lineFollowEnable
        self._arm          = armEnable
        self._sensor       = lineSensor
        self._line_err     = lineError
        self._lf_start_ms   = 0
        self._lf_print_ms   = 0      # throttle print rate during LF
        self._lf_enc_l0     = 0
        self._lf_enc_r0     = 0
        self._lf_s_hat0     = 0.0
        self._lf_s_hat_init = False  # latch s_hat on first display tick

        # motor test effort shares (optional but recommended)
        self._leftEffortCmd  = leftEffortCmd
        self._rightEffortCmd = rightEffortCmd
        self._leftEncoder    = leftEncoder
        self._rightEncoder   = rightEncoder
        self._leftMotorDrv   = leftMotorDriver
        self._rightMotorDrv  = rightMotorDriver
        self._mot_start_ms   = 0    # timer for motor test states
        self._mot_print_ms   = 0    # throttle position print rate

        # IMU shares (optional)
        self._imu_heading  = imu_heading
        self._imu_yawrate  = imu_yawrate
        self._imu_calib    = imu_calib

        # right wheel offset for straight test
        self._right_offset = float(right_offset)
        self._ctrl = ctrl

        # state estimator shares (optional)
        self._s_hat      = s_hat_share
        self._psi_hat    = psi_hat_share
        self._omL_hat    = omL_hat_share
        self._omR_hat    = omR_hat_share
        self._lf_psi_hat0 = 0.0

        # calibration state
        self._white_adc    = None
        self._black_adc    = None
        self._cal_print_ms = 0

        self._io = _IO()
        self._io.write("User Task instantiated\r\n")

    # ------------------------------------------------------------------
    def _disarm(self):
        self._arm.put(False)
        self._lf_en.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        if self._leftEffortCmd is not None:
            self._leftEffortCmd.put(0.0)
        if self._rightEffortCmd is not None:
            self._rightEffortCmd.put(0.0)

    def _print_odometry(self):
        if self._ctrl is None:
            return
        x, y, hdeg, dist = self._ctrl.get_odometry()
        self._io.write("--- Position from start ---\r\n")
        self._io.write("  X={:.1f}mm  Y={:.1f}mm  Heading={:.1f}deg\r\n".format(x, y, hdeg))
        self._io.write("  Straight-line dist={:.1f}mm\r\n".format(dist))

    def _lf_begin(self):
        self._arm.put(True)
        self._lf_en.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._lf_start_ms = ticks_ms()
        # Zero encoders now so display distances always start at 0.
        # The control task also zeros the left encoder on its next tick — that is fine.
        if self._leftEncoder  is not None: self._leftEncoder.zero()
        if self._rightEncoder is not None: self._rightEncoder.zero()
        self._lf_enc_l0     = 0
        self._lf_enc_r0     = 0
        self._lf_s_hat0     = 0.0
        self._lf_psi_hat0   = 0.0
        self._lf_s_hat_init = False  # will latch on first display tick
        self._lf_imu_psi0   = (self._imu_heading.get() or 0.0) if self._imu_heading is not None else 0.0
        self._io.write("LF ON (10 s)\r\n")
        self._io.write("t_s,err,sL_meas,sR_meas,psi_imu,pdot_imu,sL_hat,sR_hat,psi_hat,pdot_hat,omL_hat,omR_hat\r\n")
        # Drain any trailing \r or \n left in the buffer from the command
        # that triggered this (e.g. step_tool sends 'f\r\n' so the \n
        # would immediately cancel the run on the next tick).
        while self._io.any():
            self._io.read1()
        self._state = S4_LF

    def _lf_print(self, t_s, err):
        if self._leftEncoder  is not None: self._leftEncoder.update()
        if self._rightEncoder is not None: self._rightEncoder.update()
        l_pos = (self._leftEncoder.get_position()  - self._lf_enc_l0) if self._leftEncoder  is not None else 0
        r_pos = (self._rightEncoder.get_position() - self._lf_enc_r0) if self._rightEncoder is not None else 0
        sL_m = -l_pos * MM_PER_COUNT
        sR_m = -r_pos * MM_PER_COUNT
        psi_imu  = ((self._imu_heading.get() or 0.0) - self._lf_imu_psi0) * 0.017453 if self._imu_heading is not None else 0.0
        pdot_imu = (self._imu_yawrate.get() or 0.0) * 0.017453 if self._imu_yawrate is not None else 0.0
        s_hat_v   = (self._s_hat.get()   or 0.0) if self._s_hat   is not None else 0.0
        psi_hat_v = (self._psi_hat.get() or 0.0) if self._psi_hat is not None else 0.0
        omL_v     = (self._omL_hat.get() or 0.0) if self._omL_hat is not None else 0.0
        omR_v     = (self._omR_hat.get() or 0.0) if self._omR_hat is not None else 0.0
        if not self._lf_s_hat_init:
            self._lf_s_hat0     = s_hat_v
            self._lf_psi_hat0   = psi_hat_v
            self._lf_s_hat_init = True
        s_rel    = s_hat_v   - self._lf_s_hat0
        psi_rel  = psi_hat_v - self._lf_psi_hat0
        sL_hat_v = (s_rel - 0.0745 * psi_rel) * 1000.0
        sR_hat_v = (s_rel + 0.0745 * psi_rel) * 1000.0
        pdot_hat = (omR_v - omL_v) * 0.2349
        self._io.write("{:.3f},{:+.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.3f},{:.3f}\r\n".format(
            t_s, err, sL_m, sR_m, psi_imu, pdot_imu, sL_hat_v, sR_hat_v, psi_rel, pdot_hat, omL_v, omR_v))

    # ------------------------------------------------------------------
    # Motor test helpers
    # ------------------------------------------------------------------
    def _mot_test_begin(self, left: bool):
        # Full disarm: keep ALL task flags False so no other task touches motors
        self._arm.put(False)
        self._lf_en.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        if self._leftEffortCmd is not None:
            self._leftEffortCmd.put(0.0)
        if self._rightEffortCmd is not None:
            self._rightEffortCmd.put(0.0)

        if left:
            # Zero right motor effort; leave hardware state to task_motor
            if self._rightMotorDrv is not None:
                self._rightMotorDrv.set_effort(0)
            if self._leftMotorDrv is not None:
                self._leftMotorDrv.enable()
                self._leftMotorDrv.set_effort(-MOT_TEST_EFFORT)
            self._io.write("LEFT motor test ON (2 s)  x=cancel\r\n")
            self._state = S6_MOT_L
        else:
            # Zero left motor effort; leave hardware state to task_motor
            if self._leftMotorDrv is not None:
                self._leftMotorDrv.set_effort(0)
            if self._rightMotorDrv is not None:
                self._rightMotorDrv.enable()
                self._rightMotorDrv.set_effort(-MOT_TEST_EFFORT)
            self._io.write("RIGHT motor test ON (2 s)  x=cancel\r\n")
            self._state = S7_MOT_R

        self._mot_start_ms = ticks_ms()
        self._mot_print_ms = ticks_ms()
        # Drain trailing \r\n from the triggering keypress
        while self._io.any():
            self._io.read1()

    def _straight_begin(self):
        self._arm.put(False)
        self._lf_en.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        if self._leftEffortCmd is not None:
            self._leftEffortCmd.put(0.0)
        if self._rightEffortCmd is not None:
            self._rightEffortCmd.put(0.0)

        effort = float(MOT_TEST_EFFORT)
        r_offset = self._right_offset if self._right_offset is not None else 0.0
        if self._leftMotorDrv is not None:
            self._leftMotorDrv.enable()
            self._leftMotorDrv.set_effort(-effort)
        if self._rightMotorDrv is not None:
            self._rightMotorDrv.enable()
            self._rightMotorDrv.set_effort(-(effort + r_offset))

        self._io.write("STRAIGHT test ON (3 s)  x=cancel\r\n")
        self._io.write("Veers LEFT  -> increase RIGHT_OFFSET in main.py\r\n")
        self._io.write("Veers RIGHT -> decrease RIGHT_OFFSET (or go negative)\r\n")
        self._mot_start_ms = ticks_ms()
        self._mot_print_ms = ticks_ms()
        while self._io.any():
            self._io.read1()
        self._state = S8_STRAIGHT

    def _mot_test_done(self, label: str):
        # Zero effort on both drivers directly (they were driven directly during the test)
        # then hand ownership back to task_motor by zeroing all flags.
        # Do NOT call disable() here — task_motor will call _safe_off() itself
        # on the next tick when it sees arm=False/goFlag=False, keeping its
        # internal state consistent and allowing a clean restart via 'f'.
        if self._leftMotorDrv is not None:
            self._leftMotorDrv.set_effort(0)
        if self._rightMotorDrv is not None:
            self._rightMotorDrv.set_effort(0)
        self._arm.put(False)
        self._lf_en.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)
        if self._leftEffortCmd is not None:
            self._leftEffortCmd.put(0.0)
        if self._rightEffortCmd is not None:
            self._rightEffortCmd.put(0.0)
        self._io.write("\r\n{} test DONE  f=start  l=left  r=right  c=calibrate\r\n".format(label) + UI_prompt)
        self._state = S1_CMD

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------
    def _cal_enter(self):
        self._disarm()
        self._white_adc = None
        self._black_adc = None
        self._cal_print_ms = ticks_ms() - CAL_PRINT_MS  # print immediately
        self._io.write("\r\n--- CALIBRATION MODE ---\r\n")
        self._io.write("w=capture white  b=capture black  x=exit\r\n")
        # Drain trailing \r\n from the 'c' command
        while self._io.any():
            self._io.read1()
        self._state = S5_CAL

    def _cal_print_live(self, readings):
        # compute error using current cal values (or raw if not calibrated)
        white = self._white_adc if self._white_adc is not None else min(readings)
        black = self._black_adc if self._black_adc is not None else max(readings)
        scale = black - white
        if scale == 0:
            error = 0.0
        else:
            weights = [i - (N_SENSORS - 1) / 2.0 for i in range(N_SENSORS)]
            num = 0.0
            den = 0.0
            for w, r in zip(weights, readings):
                n = max(0.0, min(1.0, (r - white) / scale))
                num += w * n
                den += n
            error = (num / den) if den > 0.01 else 0.0

        # status line: W/B capture state
        w_str = str(self._white_adc) if self._white_adc is not None else "???"
        b_str = str(self._black_adc) if self._black_adc is not None else "???"
        adc_str = ",".join(str(int(r)) for r in readings)
        self._io.write("W={} B={} ERR={:+.3f} | {}\r\n".format(
            w_str, b_str, error, adc_str))

    def _cal_capture_white(self, readings):
        self._white_adc = int(sum(readings) / len(readings))
        self._io.write(">> WHITE captured: {}\r\n".format(self._white_adc))
        self._cal_try_recommend()

    def _cal_capture_black(self, readings):
        self._black_adc = int(max(readings))
        self._io.write(">> BLACK captured: {}\r\n".format(self._black_adc))
        self._cal_try_recommend()

    def _cal_try_recommend(self):
        if self._white_adc is None or self._black_adc is None:
            return
        w = self._white_adc
        b = self._black_adc
        contrast = abs(b - w) / 4095.0 * 100.0
        self._io.write("\r\n=== CAL RESULTS ===\r\n")
        self._io.write("white_adc={} black_adc={} contrast={:.1f}%\r\n".format(w, b, contrast))
        self._io.write("Copy into main.py: black_adc={} white_adc={}\r\n".format(b, w))
        self._io.write("Press x to exit.\r\n")

    # ------------------------------------------------------------------
    # Main generator
    # ------------------------------------------------------------------
    def run(self):
        while True:

            # ---- S0: init ----
            if self._state == S0_INIT:
                self._disarm()
                self._io.write("\r\nREADY  f=line  l=left  r=right  s=straight  c=cal\r\n" + UI_prompt)
                self._state = S1_CMD

            # ---- S1: waiting for keypress ----
            elif self._state == S1_CMD:
                if self._io.any():
                    b = self._io.read1()
                    if b:
                        try:
                            ch = b.decode()
                        except Exception:
                            ch = ""

                        if ch in {"f", "F"}:
                            self._lf_begin()
                        elif ch in {"l", "L"}:
                            self._mot_test_begin(left=True)
                        elif ch in {"r", "R"}:
                            self._mot_test_begin(left=False)
                        elif ch in {"s", "S"}:
                            self._straight_begin()
                        elif ch in {"c", "C"}:
                            self._cal_enter()
                        else:
                            self._disarm()
                            self._io.write("\r\nSTOPPED  f=line  l=left  r=right  s=straight  c=cal\r\n" + UI_prompt)

                        yield self._state
                        continue

            # ---- S4: line-follow timed run ----
            elif self._state == S4_LF:
                elapsed = ticks_diff(ticks_ms(), self._lf_start_ms)
                if self._io.any():
                    self._io.read1()   # consume key
                    self._disarm()
                    self._print_odometry()
                    self._io.write("\r\nLF CANCELLED  f=line  c=cal\r\n" + UI_prompt)
                    self._state = S1_CMD
                    yield self._state
                    continue

                # Print line error + encoder data at ~50 ms intervals
                now_ms = ticks_ms()
                if ticks_diff(now_ms, self._lf_print_ms) >= 50:
                    self._lf_print_ms = now_ms
                    t_s = elapsed / 1000.0
                    err = self._line_err.get() if self._line_err is not None else 0.0

                    self._lf_print(t_s, err)

                if elapsed >= LF_DURATION_MS:
                    self._disarm()
                    self._print_odometry()
                    self._io.write("\r\nLF DONE (10 s)  f=line  c=cal\r\n" + UI_prompt)
                    self._state = S1_CMD

            # ---- S6: left motor test ----
            elif self._state == S6_MOT_L:
                if self._io.any():
                    self._io.read1()
                    self._mot_test_done("LEFT motor")
                    yield self._state
                    continue

                # Print both encoder positions at ~100 ms intervals
                now_ms = ticks_ms()
                if ticks_diff(now_ms, self._mot_print_ms) >= 100:
                    self._mot_print_ms = now_ms
                    elapsed = ticks_diff(now_ms, self._mot_start_ms)
                    if self._leftEncoder is not None:
                        self._leftEncoder.update()
                    if self._rightEncoder is not None:
                        self._rightEncoder.update()
                    l_pos = self._leftEncoder.get_position()  if self._leftEncoder  is not None else 0
                    r_pos = self._rightEncoder.get_position() if self._rightEncoder is not None else 0
                    l_vel = self._leftEncoder.get_velocity()  if self._leftEncoder  is not None else 0
                    r_vel = self._rightEncoder.get_velocity() if self._rightEncoder is not None else 0
                    l_raw = self._leftEncoder.tim_pin.counter()  if self._leftEncoder  is not None else 0
                    r_raw = self._rightEncoder.tim_pin.counter() if self._rightEncoder is not None else 0
                    self._io.write("L=({},{:.0f},{}) R=({},{:.0f},{}) t={:.2f}s\r\n".format(
                        l_pos, l_vel, l_raw, r_pos, r_vel, r_raw, elapsed / 1000.0))

                if ticks_diff(ticks_ms(), self._mot_start_ms) >= MOT_TEST_MS:
                    self._mot_test_done("LEFT motor")

            # ---- S7: right motor test ----
            elif self._state == S7_MOT_R:
                if self._io.any():
                    self._io.read1()
                    self._mot_test_done("RIGHT motor")
                    yield self._state
                    continue

                # Print both encoder positions at ~100 ms intervals
                now_ms = ticks_ms()
                if ticks_diff(now_ms, self._mot_print_ms) >= 100:
                    self._mot_print_ms = now_ms
                    elapsed = ticks_diff(now_ms, self._mot_start_ms)
                    if self._leftEncoder is not None:
                        self._leftEncoder.update()
                    if self._rightEncoder is not None:
                        self._rightEncoder.update()
                    l_pos = self._leftEncoder.get_position()  if self._leftEncoder  is not None else 0
                    r_pos = self._rightEncoder.get_position() if self._rightEncoder is not None else 0
                    l_vel = self._leftEncoder.get_velocity()  if self._leftEncoder  is not None else 0
                    r_vel = self._rightEncoder.get_velocity() if self._rightEncoder is not None else 0
                    l_raw = self._leftEncoder.tim_pin.counter()  if self._leftEncoder  is not None else 0
                    r_raw = self._rightEncoder.tim_pin.counter() if self._rightEncoder is not None else 0
                    self._io.write("L=({},{:.0f},{}) R=({},{:.0f},{}) t={:.2f}s\r\n".format(
                        l_pos, l_vel, l_raw, r_pos, r_vel, r_raw, elapsed / 1000.0))

                if ticks_diff(ticks_ms(), self._mot_start_ms) >= MOT_TEST_MS:
                    self._mot_test_done("RIGHT motor")

            # ---- S8: straight-line trim test ----
            elif self._state == S8_STRAIGHT:
                if self._io.any():
                    self._io.read1()
                    self._mot_test_done("STRAIGHT")
                    yield self._state
                    continue

                # Print both encoder positions at ~100 ms so you can see drift
                now_ms = ticks_ms()
                if ticks_diff(now_ms, self._mot_print_ms) >= 100:
                    self._mot_print_ms = now_ms
                    elapsed = ticks_diff(now_ms, self._mot_start_ms)
                    l_pos = self._leftEncoder.get_position()  if self._leftEncoder  is not None else 0
                    r_pos = self._rightEncoder.get_position() if self._rightEncoder is not None else 0
                    self._io.write("t={:.2f}s  L={:.2f}in  R={:.2f}in  diff={:.2f}in\r\n".format(
                        elapsed / 1000.0, l_pos, r_pos, l_pos - r_pos))

                if ticks_diff(ticks_ms(), self._mot_start_ms) >= STRAIGHT_TEST_MS:
                    self._mot_test_done("STRAIGHT")

            # ---- S5: calibration mode ----
            elif self._state == S5_CAL:

                # check for keypress
                if self._io.any():
                    b = self._io.read1()
                    if b:
                        try:
                            ch = b.decode()
                        except Exception:
                            ch = ""
                        readings = self._sensor.get_raw_readings()
                        if ch in {"w", "W"}:
                            self._cal_capture_white(readings)
                        elif ch in {"b", "B"}:
                            self._cal_capture_black(readings)
                        elif ch in {"x", "X", "q", "Q"}:
                            self._io.write("\r\nExiting calibration.\r\n" + UI_prompt)
                            self._state = S1_CMD
                            yield self._state
                            continue

                # print live readings periodically
                now = ticks_ms()
                if ticks_diff(now, self._cal_print_ms) >= CAL_PRINT_MS:
                    self._cal_print_ms = now
                    readings = self._sensor.get_raw_readings()
                    self._cal_print_live(readings)

            yield self._state
