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
S9_IMU       = micropython.const(9)  # IMU live readout (until x)

LF_DURATION_MS      = micropython.const(10_000)  # 10 seconds
MOT_TEST_MS         = micropython.const(2_000)   # 2 seconds
STRAIGHT_TEST_MS    = micropython.const(3_000)   # 3 seconds
IMU_PRINT_MS        = micropython.const(100)     # IMU print interval (10 Hz)
MOT_TEST_EFFORT     = micropython.const(30)      # PWM % for motor test
CAL_PRINT_MS        = micropython.const(200)     # print interval during cal
MM_PER_COUNT        = 0.15272                    # wheel radius 35mm, 1440 CPR

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
    """
    UI task.
      f          = start line follow (10 s auto-stop)
      l          = spin LEFT motor only for 2 s (motor test)
      r          = spin RIGHT motor only for 2 s (motor test)
      x / any    = cancel / stop
      c          = enter calibration mode
        w        = capture white (average of current readings)
        b        = capture black (max of current readings)
        x        = exit calibration
    """

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
                 ctrl=None):                    # control task object for odometry readout
        self._state        = S0_INIT
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._lf_en        = lineFollowEnable
        self._arm          = armEnable
        self._sensor       = lineSensor
        self._line_err     = lineError
        self._lf_start_ms  = 0
        self._lf_print_ms  = 0      # throttle print rate during LF

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
        self._imu_start_ms = 0
        self._imu_print_ms = 0

        # right wheel offset for straight test
        self._right_offset = float(right_offset)
        self._ctrl = ctrl

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
        """Print final position after a run."""
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
        self._io.write("LF ON (10 s)\r\n")
        # Drain any trailing \r or \n left in the buffer from the command
        # that triggered this (e.g. step_tool sends 'f\r\n' so the \n
        # would immediately cancel the run on the next tick).
        while self._io.any():
            self._io.read1()
        self._state = S4_LF

    # ------------------------------------------------------------------
    # Motor test helpers
    # ------------------------------------------------------------------
    def _mot_test_begin(self, left: bool):
        """Start a 2-second single-motor spin test.

        Drives the motor hardware DIRECTLY (enable + set_effort) so the
        control task cannot interfere.  Both go-flags are kept False so
        task_control stays in S1_WAIT and task_motor stays in S1_WAIT —
        neither will touch the hardware while we own it here.
        """
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

    def _imu_begin(self):
        """Stream IMU heading, yaw rate, and calibration status until x is pressed."""
        if self._imu_heading is None:
            self._io.write("IMU shares not connected — pass imu_heading= to task_user\r\n" + UI_prompt)
            return
        self._imu_start_ms = ticks_ms()
        self._imu_print_ms = ticks_ms() - IMU_PRINT_MS  # print immediately on first tick
        self._io.write("\r\n--- IMU READOUT (press x to stop) ---\r\n")
        self._io.write("time(s)   heading(deg)  yaw_rate(dps)  calib(sys/gyr/acc/mag)\r\n")
        while self._io.any():
            self._io.read1()
        self._state = S9_IMU

    def _straight_begin(self):
        """Run both motors for 3 s with no steering so you can tune RIGHT_OFFSET.

        Left  motor gets -MOT_TEST_EFFORT.
        Right motor gets -(MOT_TEST_EFFORT + right_offset) matching what the
        control task does, so what you see here is what you get during line follow.
        """
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
        """Print ADC readings and current error on one line."""
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
        """If both white and black are captured, print recommended K values."""
        if self._white_adc is None or self._black_adc is None:
            return
        w = self._white_adc
        b = self._black_adc
        scale = b - w
        contrast = abs(scale) / 4095.0 * 100.0
        self._io.write("\r\n=== CALIBRATION RESULTS ===\r\n")
        self._io.write("white_adc = {}\r\n".format(w))
        self._io.write("black_adc = {}\r\n".format(b))
        self._io.write("contrast  = {:.1f}%  ".format(contrast))
        if contrast > 40:
            self._io.write("(excellent)\r\n")
        elif contrast > 20:
            self._io.write("(good)\r\n")
        elif contrast > 10:
            self._io.write("(marginal - consider recalibrating)\r\n")
        else:
            self._io.write("(TOO LOW - recalibrate on better surface)\r\n")

        self._io.write("\r\n--- Recommended STEER_GAIN values ---\r\n")
        self._io.write("(based on max error = 5.0, adjust BASE_VEL column for your speed)\r\n")
        self._io.write("{:<12} {:<14} {}\r\n".format("STEER_GAIN", "Δvel @BASE400", "Behaviour"))
        self._io.write("{}\r\n".format("-" * 44))
        gains = [
            (0.02, "conservative"),
            (0.04, "conservative"),
            (0.08, "moderate    "),
            (0.12, "moderate    "),
            (0.18, "aggressive  "),
            (0.25, "aggressive  "),
        ]
        for k, label in gains:
            delta = k * 5.0 * 400
            pct   = delta / 400 * 100
            rec   = " <-- recommended" if 30 <= pct <= 60 else ""
            self._io.write("{:<12} {:<14} {}{}\r\n".format(
                str(k), "+/-{:.0f} ({:.0f}%)".format(delta, pct), label, rec))

        self._io.write("\r\nCopy into main.py:\r\n")
        self._io.write("  black_adc  = {}\r\n".format(b))
        self._io.write("  white_adc  = {}\r\n".format(w))
        self._io.write("\r\nPress x to exit calibration.\r\n")

    # ------------------------------------------------------------------
    # Main generator
    # ------------------------------------------------------------------
    def run(self):
        while True:

            # ---- S0: init ----
            if self._state == S0_INIT:
                self._disarm()
                self._io.write("\r\nREADY  f=line  l=left  r=right  s=straight  i=imu  c=cal\r\n" + UI_prompt)
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
                        elif ch in {"i", "I"}:
                            self._imu_begin()
                        elif ch in {"c", "C"}:
                            self._cal_enter()
                        else:
                            self._disarm()
                            self._io.write("\r\nSTOPPED  f=line  l=left  r=right  s=straight  i=imu  c=cal\r\n" + UI_prompt)

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

                    # read encoder positions (update for fresh reading)
                    if self._leftEncoder is not None:
                        self._leftEncoder.update()
                    if self._rightEncoder is not None:
                        self._rightEncoder.update()
                    l_pos = self._leftEncoder.get_position()  if self._leftEncoder  is not None else 0
                    r_pos = self._rightEncoder.get_position() if self._rightEncoder is not None else 0
                    r_pos = self._rightEncoder.get_position() if self._rightEncoder is not None else 0
                    l_mm = l_pos * MM_PER_COUNT
                    r_mm = -r_pos * MM_PER_COUNT  # negate: right encoder counts negative going forward

                    self._io.write("LF,{:.3f},err={:+.4f},Lmm={:.1f},Rmm={:.1f}\r\n".format(
                        t_s, err, l_mm, r_mm))

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

                # Print left encoder position at ~100 ms intervals
                if self._leftEncoder is not None:
                    now_ms = ticks_ms()
                    if ticks_diff(now_ms, self._mot_print_ms) >= 100:
                        self._mot_print_ms = now_ms
                        self._leftEncoder.update()
                        elapsed = ticks_diff(now_ms, self._mot_start_ms)
                        self._io.write("L pos={} vel={:.0f} t={:.2f}s\r\n".format(
                            self._leftEncoder.get_position(),
                            self._leftEncoder.get_velocity(),
                            elapsed / 1000.0))

                if ticks_diff(ticks_ms(), self._mot_start_ms) >= MOT_TEST_MS:
                    self._mot_test_done("LEFT motor")

            # ---- S7: right motor test ----
            elif self._state == S7_MOT_R:
                if self._io.any():
                    self._io.read1()
                    self._mot_test_done("RIGHT motor")
                    yield self._state
                    continue

                # Print right encoder position at ~100 ms intervals
                if self._rightEncoder is not None:
                    now_ms = ticks_ms()
                    if ticks_diff(now_ms, self._mot_print_ms) >= 100:
                        self._mot_print_ms = now_ms
                        self._rightEncoder.update()
                        elapsed = ticks_diff(now_ms, self._mot_start_ms)
                        self._io.write("R pos={} vel={:.0f} t={:.2f}s\r\n".format(
                            self._rightEncoder.get_position(),
                            self._rightEncoder.get_velocity(),
                            elapsed / 1000.0))

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

            # ---- S9: IMU live readout ----
            elif self._state == S9_IMU:
                if self._io.any():
                    self._io.read1()
                    self._io.write("\r\nIMU readout stopped.\r\n" + UI_prompt)
                    self._state = S1_CMD
                    yield self._state
                    continue

                now_ms = ticks_ms()
                if ticks_diff(now_ms, self._imu_print_ms) >= IMU_PRINT_MS:
                    self._imu_print_ms = now_ms
                    t_s = ticks_diff(now_ms, self._imu_start_ms) / 1000.0
                    heading  = self._imu_heading.get()  if self._imu_heading  is not None else 0.0
                    yawrate  = self._imu_yawrate.get()  if self._imu_yawrate  is not None else 0.0
                    cal_byte = self._imu_calib.get()    if self._imu_calib    is not None else 0
                    sys_cal = (cal_byte >> 6) & 0x03
                    gyr_cal = (cal_byte >> 4) & 0x03
                    acc_cal = (cal_byte >> 2) & 0x03
                    mag_cal = (cal_byte >> 0) & 0x03
                    self._io.write("{:7.2f}   {:+8.2f}      {:+8.2f}       {}/{}/{}/{}\r\n".format(
                        t_s, heading, yawrate, sys_cal, gyr_cal, acc_cal, mag_cal))

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
