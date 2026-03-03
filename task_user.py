# task_ui.py
# Merged task_user + task_bt into a single UI task.
#
# - USB serial (sys.stdin/stdout) and Bluetooth (BTConsole) share the same FSM.
# - Every message is sent to BOTH outputs simultaneously.
# - step_tool.py works over BT just as before (looks for LF,t,err,imu_h,imu_wz lines).
# - REC mode (record to RAM, REC DUMP) works over BT only.
# - SET/GET/PING commands work over BT only.
#
# FSM states:
#   S0_INIT  - first tick: disarm, print READY
#   S1_CMD   - waiting for a command
#   S4_LF    - line follow running (10 s auto-stop)
#   S5_CAL   - calibration mode

import sys
import uselect
import micropython
import pyb
from utime import ticks_ms, ticks_diff
from task_share import Share

S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)
S4_LF   = micropython.const(4)
S5_CAL  = micropython.const(5)

LF_DURATION_MS = micropython.const(10_000)
CAL_PRINT_MS   = micropython.const(200)
LF_PRINT_MS    = micropython.const(50)

UI_PROMPT  = ">: "
N_SENSORS  = 11


# ---------------------------------------------------------------------------
# USB serial helper (non-blocking single-char read)
# ---------------------------------------------------------------------------
class _USBio:
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

    def drain(self):
        while self.any():
            self.read1()


# ---------------------------------------------------------------------------
# Main merged UI task
# ---------------------------------------------------------------------------
class task_user:
    """
    Single UI task that mirrors all output to both USB serial and Bluetooth.

    Parameters
    ----------
    bt : BTConsole
        Bluetooth console object (bt_console.BTConsole).
    leftMotorGo, rightMotorGo : Share
    lineFollowEnable : Share
    armEnable : Share
    lineSensor : L_sensor
    lineError : Share, optional
    imu_heading : Share, optional
    imu_yawrate : Share, optional
    imu_calib : Share, optional
    ctrl_obj : task_control, optional
        Used by SET/GET BT commands to read/write gains.
    baseEffort_share : Share, optional
    observer_states : Share, optional
        Share holding [vL, vR, heading, yawrate] from observer task.
    """

    def __init__(self,
                 bt,
                 leftMotorGo: Share,
                 rightMotorGo: Share,
                 lineFollowEnable: Share,
                 armEnable: Share,
                 lineSensor,
                 lineError: Share = None,
                 imu_heading: Share = None,
                 imu_yawrate: Share = None,
                 imu_calib: Share = None,
                 ctrl_obj=None,
                 baseEffort_share: Share = None,
                 observer_states: Share = None):

        self._state        = S0_INIT
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._lf_en        = lineFollowEnable
        self._arm          = armEnable
        self._sensor       = lineSensor
        self._line_err     = lineError
        self._imu_heading  = imu_heading
        self._imu_yawrate  = imu_yawrate
        self._imu_calib    = imu_calib
        self._ctrl         = ctrl_obj
        self._base_effort  = baseEffort_share
        self._obs_states   = observer_states

        self._lf_start_ms  = 0
        self._lf_print_ms  = 0
        self._cal_print_ms = 0

        # calibration state
        self._white_adc = None
        self._black_adc = None

        # I/O handles
        self._usb = _USBio()
        self._bt  = bt          # BTConsole instance

        # BT record mode
        self._rec_recording      = False
        self._rec_period_ms      = 20
        self._rec_target_samples = 0
        self._rec_samples        = []
        self._rec_max_samples    = 1000
        self._rec_last_ms        = 0

        # BT live telemetry
        self._telem_period_ms = 200   # 5 Hz default
        self._telem_last_ms   = 0

        self._out("User Task instantiated\r\n")

    # ------------------------------------------------------------------
    # Dual output helper — sends to both USB and BT
    # ------------------------------------------------------------------
    def _out(self, s):
        self._usb.write(s)
        try:
            # BTConsole.send_line adds \r\n, so strip trailing \r\n first
            line = s.rstrip("\r\n")
            if line:
                self._bt.send_line(line)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Motor control helpers
    # ------------------------------------------------------------------
    def _disarm(self):
        self._arm.put(False)
        self._lf_en.put(False)
        self._leftMotorGo.put(False)
        self._rightMotorGo.put(False)

    def _lf_begin(self):
        self._arm.put(True)
        self._lf_en.put(True)
        self._leftMotorGo.put(True)
        self._rightMotorGo.put(True)
        self._lf_start_ms = ticks_ms()
        self._lf_print_ms = self._lf_start_ms - LF_PRINT_MS
        self._out("LF ON (10 s)\r\n")
        # Drain any trailing \r\n that arrived with the 'f' command
        self._usb.drain()
        self._state = S4_LF

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------
    def _cal_enter(self):
        self._disarm()
        self._white_adc    = None
        self._black_adc    = None
        self._cal_print_ms = ticks_ms() - CAL_PRINT_MS
        self._out("\r\n--- CALIBRATION MODE ---\r\n")
        self._out("w=capture white  b=capture black  x=exit\r\n")
        self._usb.drain()
        self._state = S5_CAL

    def _cal_print_live(self, readings):
        white = self._white_adc if self._white_adc is not None else min(readings)
        black = self._black_adc if self._black_adc is not None else max(readings)
        scale = black - white
        if scale == 0:
            error = 0.0
        else:
            weights = [i - (N_SENSORS - 1) / 2.0 for i in range(N_SENSORS)]
            num = den = 0.0
            for w, r in zip(weights, readings):
                n = max(0.0, min(1.0, (r - white) / scale))
                num += w * n
                den += n
            error = (num / den) if den > 0.01 else 0.0
        w_str   = str(self._white_adc) if self._white_adc is not None else "???"
        b_str   = str(self._black_adc) if self._black_adc is not None else "???"
        adc_str = ",".join(str(int(r)) for r in readings)
        self._out("W={} B={} ERR={:+.3f} | {}\r\n".format(w_str, b_str, error, adc_str))

    def _cal_capture_white(self, readings):
        self._white_adc = int(sum(readings) / len(readings))
        self._out(">> WHITE captured: {}\r\n".format(self._white_adc))
        self._cal_try_recommend()

    def _cal_capture_black(self, readings):
        self._black_adc = int(max(readings))
        self._out(">> BLACK captured: {}\r\n".format(self._black_adc))
        self._cal_try_recommend()

    def _cal_try_recommend(self):
        if self._white_adc is None or self._black_adc is None:
            return
        w = self._white_adc
        b = self._black_adc
        contrast = abs(b - w) / 4095.0 * 100.0
        self._out("\r\n=== CALIBRATION RESULTS ===\r\n")
        self._out("white_adc = {}\r\n".format(w))
        self._out("black_adc = {}\r\n".format(b))
        self._out("contrast  = {:.1f}%  ".format(contrast))
        if contrast > 40:
            self._out("(excellent)\r\n")
        elif contrast > 20:
            self._out("(good)\r\n")
        elif contrast > 10:
            self._out("(marginal - consider recalibrating)\r\n")
        else:
            self._out("(TOO LOW - recalibrate on better surface)\r\n")
        self._out("\r\n--- Recommended STEER_GAIN values ---\r\n")
        self._out("(based on max error = 5.0)\r\n")
        gains = [(0.02,"conservative"),(0.04,"conservative"),(0.08,"moderate    "),
                 (0.12,"moderate    "),(0.18,"aggressive  "),(0.25,"aggressive  ")]
        for k, label in gains:
            delta = k * 5.0 * 400
            pct   = delta / 400 * 100
            rec   = " <-- recommended" if 30 <= pct <= 60 else ""
            self._out("{:<12} +/-{:.0f} ({:.0f}%)  {}{}\r\n".format(k, delta, pct, label, rec))
        self._out("\r\nCopy into main.py:\r\n")
        self._out("  black_adc = {}\r\n".format(b))
        self._out("  white_adc = {}\r\n".format(w))
        self._out("\r\nPress x to exit calibration.\r\n")

    # ------------------------------------------------------------------
    # BT-only: data snapshot for telemetry / record mode
    # ------------------------------------------------------------------
    def _read_snapshot(self):
        t       = pyb.millis()
        line_err = 0.0
        try:
            line_err = float(self._sensor.calculate_error())
        except Exception:
            pass
        imu_h   = self._imu_heading.get()  if self._imu_heading  is not None else None
        imu_wz  = self._imu_yawrate.get()  if self._imu_yawrate  is not None else None
        imu_cal = self._imu_calib.get()    if self._imu_calib    is not None else None
        obs_vL = obs_vR = obs_h = obs_wz = None
        if self._obs_states is not None:
            try:
                obs = self._obs_states.get()
                if obs is not None and len(obs) >= 4:
                    obs_vL, obs_vR, obs_h, obs_wz = obs[0], obs[1], obs[2], obs[3]
            except Exception:
                pass
        base    = self._base_effort.get() if self._base_effort is not None else None
        kp_line = getattr(self._ctrl, "_kp_line",    None)
        kd_line = getattr(self._ctrl, "_kd_line",    None)
        max_eff = getattr(self._ctrl, "_max_effort", None)
        return (t, line_err, imu_h, imu_wz, imu_cal,
                obs_vL, obs_vR, obs_h, obs_wz,
                base, kp_line, kd_line, max_eff)

    def _send_telemetry(self):
        (t, line_err, imu_h, imu_wz, imu_cal,
         obs_vL, obs_vR, obs_h, obs_wz,
         base, kp_line, kd_line, max_eff) = self._read_snapshot()
        try:
            self._bt.send_line(
                "T t={} line_err={:.3f} imu_h={} imu_wz={} imu_cal={} "
                "obs_vL={} obs_vR={} obs_h={} obs_wz={} "
                "base={} kp={}".format(
                    t, line_err, imu_h, imu_wz, imu_cal,
                    obs_vL, obs_vR, obs_h, obs_wz,
                    base, kp_line))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # BT-only: record mode
    # ------------------------------------------------------------------
    def _rec_start(self, hz, seconds=None, samples=None):
        if hz <= 0:
            self._bt.send_line("ERR hz must be > 0")
            return
        self._rec_period_ms = max(1, int(1000 / hz))
        self._rec_samples   = []
        target = int(samples) if samples is not None else int(hz * seconds)
        if target <= 0:
            self._bt.send_line("ERR target must be > 0")
            return
        if target > self._rec_max_samples:
            target = self._rec_max_samples
            self._bt.send_line("WARN capped to {} samples".format(self._rec_max_samples))
        self._rec_target_samples = target
        self._rec_recording  = True
        self._rec_last_ms    = pyb.millis()
        self._bt.send_line("OK REC START hz={} period_ms={} target={}".format(
            hz, self._rec_period_ms, target))

    def _rec_stop(self):
        self._rec_recording = False
        self._bt.send_line("OK REC STOP n={}".format(len(self._rec_samples)))

    def _rec_clear(self):
        self._rec_recording      = False
        self._rec_samples        = []
        self._rec_target_samples = 0
        self._bt.send_line("OK REC CLEAR")

    def _rec_status(self):
        self._bt.send_line("REC recording={} n={} target={} period_ms={} max={}".format(
            int(self._rec_recording), len(self._rec_samples),
            self._rec_target_samples, self._rec_period_ms, self._rec_max_samples))

    def _rec_dump(self):
        self._bt.send_line("CSV t_ms,line_err,imu_heading,imu_yawrate,imu_calib,"
                           "obs_vL,obs_vR,obs_heading,obs_yawrate,"
                           "base,kp_line,kd_line,max_eff")
        for s in self._rec_samples:
            (t, line_err, imu_h, imu_wz, imu_cal,
             obs_vL, obs_vR, obs_h, obs_wz,
             base, kp_line, kd_line, max_eff) = s
            self._bt.send_line(
                "{},{:.6f},{},{},{},{},{},{},{},{},{},{},{}".format(
                    t, line_err, imu_h, imu_wz, imu_cal,
                    obs_vL, obs_vR, obs_h, obs_wz,
                    base, kp_line, kd_line, max_eff))
        self._bt.send_line("OK REC DUMP n={}".format(len(self._rec_samples)))

    # ------------------------------------------------------------------
    # BT-only: handle line commands (PING, GET, SET, REC)
    # Returns True if the line was consumed as a debug command.
    # ------------------------------------------------------------------
    def _handle_bt_cmd(self, line):
        parts = line.split()
        if not parts:
            return False
        cmd = parts[0].upper()

        if cmd == "PING":
            self._bt.send_line("PONG")
            return True

        if cmd == "GET":
            self._send_telemetry()
            return True

        if cmd == "SET" and len(parts) == 3:
            key = parts[1].lower()
            try:
                val = float(parts[2])
            except Exception:
                self._bt.send_line("ERR value must be numeric")
                return True
            if key == "base":
                if self._base_effort is not None:
                    self._base_effort.put(val)
            elif key == "kp_line" and self._ctrl is not None:
                self._ctrl._kp_line = val
            elif key == "kd_line" and self._ctrl is not None:
                self._ctrl._kd_line = val
            elif key == "max_effort" and self._ctrl is not None:
                self._ctrl._max_effort = val
            elif key == "hz":
                if val > 0:
                    self._telem_period_ms = int(1000 / val)
            else:
                self._bt.send_line("ERR unknown key")
                return True
            self._bt.send_line("OK")
            return True

        if cmd == "REC":
            if len(parts) < 2:
                self._bt.send_line("ERR usage: REC <START|STOP|DUMP|CLEAR|STATUS>")
                return True
            sub = parts[1].upper()
            if sub == "START":
                if len(parts) < 4:
                    self._bt.send_line("ERR usage: REC START <hz> <seconds>  OR  REC START <hz> <samples> SAMPLES")
                    return True
                hz    = float(parts[2])
                third = parts[3]
                if len(parts) >= 5 and parts[4].upper() == "SAMPLES":
                    self._rec_start(hz, samples=int(third))
                else:
                    self._rec_start(hz, seconds=float(third))
            elif sub == "STOP":   self._rec_stop()
            elif sub == "DUMP":   self._rec_dump()
            elif sub == "CLEAR":  self._rec_clear()
            elif sub == "STATUS": self._rec_status()
            else:
                self._bt.send_line("ERR unknown REC subcmd")
            return True

        return False

    # ------------------------------------------------------------------
    # Shared character handler (same logic for USB char or BT char)
    # ------------------------------------------------------------------
    def _handle_char(self, ch):
        if self._state == S1_CMD:
            if ch in {"f", "F"}:
                self._lf_begin()
            elif ch in {"c", "C"}:
                self._cal_enter()
            else:
                self._disarm()
                self._out("\r\nSTOPPED  f=start  c=calibrate\r\n" + UI_PROMPT)

        elif self._state == S4_LF:
            self._disarm()
            self._out("\r\nLF CANCELLED  f=start  c=calibrate\r\n" + UI_PROMPT)
            self._state = S1_CMD

        elif self._state == S5_CAL:
            readings = self._sensor.get_raw_readings()
            if ch in {"w", "W"}:
                self._cal_capture_white(readings)
            elif ch in {"b", "B"}:
                self._cal_capture_black(readings)
            elif ch in {"x", "X", "q", "Q"}:
                self._out("\r\nExiting calibration.\r\n" + UI_PROMPT)
                self._state = S1_CMD

    # ------------------------------------------------------------------
    # Main FSM generator  (register as a single cotask)
    # ------------------------------------------------------------------
    def run(self):
        while True:
            now_ms = ticks_ms()
            now_pyb = pyb.millis()

            # ---- S0: init ----
            if self._state == S0_INIT:
                self._disarm()
                self._out("\r\nREADY  f=start  c=calibrate\r\n" + UI_PROMPT)
                self._telem_last_ms = now_pyb
                self._rec_last_ms   = now_pyb
                self._state = S1_CMD

            # ---- Poll BT console (assembles UART bytes into lines) ----
            try:
                self._bt.poll()
            except Exception:
                pass

            # ---- Incoming BT line commands ----
            try:
                while self._bt.any_line():
                    line = self._bt.get_line()
                    if not line:
                        continue
                    # Try debug/record commands first
                    if not self._handle_bt_cmd(line):
                        # Otherwise treat first char as UI command
                        ch = line.strip()[:1]
                        if ch:
                            self._handle_char(ch)
            except Exception:
                pass

            # ---- Incoming USB char ----
            if self._usb.any():
                raw = self._usb.read1()
                if raw:
                    try:
                        ch = raw.decode()
                    except Exception:
                        ch = ""
                    if ch:
                        self._handle_char(ch)

            # ---- S4: line-follow timed behaviors ----
            if self._state == S4_LF:
                elapsed = ticks_diff(now_ms, self._lf_start_ms)

                # Print LF,t,err,imu_h,imu_wz at ~50 ms for step_tool
                if ticks_diff(now_ms, self._lf_print_ms) >= LF_PRINT_MS:
                    self._lf_print_ms = now_ms
                    err   = self._line_err.get() if self._line_err is not None else 0.0
                    imu_h = self._imu_heading.get() if self._imu_heading is not None else None
                    imu_w = self._imu_yawrate.get() if self._imu_yawrate is not None else None
                    t_s   = elapsed / 1000.0
                    self._out("LF,{:.3f},{:.4f},{},{}\r\n".format(t_s, err, imu_h, imu_w))

                if elapsed >= LF_DURATION_MS:
                    self._disarm()
                    self._out("\r\nLF DONE (10 s)  f=start  c=calibrate\r\n" + UI_PROMPT)
                    self._state = S1_CMD

            # ---- S5: calibration live print ----
            elif self._state == S5_CAL:
                if ticks_diff(now_ms, self._cal_print_ms) >= CAL_PRINT_MS:
                    self._cal_print_ms = now_ms
                    self._cal_print_live(self._sensor.get_raw_readings())

            # ---- BT record sampling (independent of UI state) ----
            if self._rec_recording:
                if pyb.elapsed_millis(self._rec_last_ms) >= self._rec_period_ms:
                    self._rec_last_ms = now_pyb
                    if len(self._rec_samples) < self._rec_target_samples:
                        self._rec_samples.append(self._read_snapshot())
                    else:
                        self._rec_stop()

            # ---- BT live telemetry (independent of UI state) ----
            if pyb.elapsed_millis(self._telem_last_ms) >= self._telem_period_ms:
                self._telem_last_ms = now_pyb
                self._send_telemetry()

            yield self._state
