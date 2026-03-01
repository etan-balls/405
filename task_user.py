# task_user.py - f=line follow (10s), x=cancel, c=calibrate

import sys
import uselect
import micropython
from utime import ticks_ms, ticks_diff
from task_share import Share

S0_INIT = micropython.const(0)
S1_CMD  = micropython.const(1)
S4_LF   = micropython.const(4)
S5_CAL  = micropython.const(5)   # calibration mode

LF_DURATION_MS  = micropython.const(10_000)  # 10 seconds
CAL_PRINT_MS    = micropython.const(200)      # print interval during cal

UI_prompt = ">: "
N_SENSORS = 11


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
                 lineError: Share = None):   # for live centroid printing during LF
        self._state        = S0_INIT
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._lf_en        = lineFollowEnable
        self._arm          = armEnable
        self._sensor       = lineSensor
        self._line_err     = lineError
        self._lf_start_ms  = 0
        self._lf_print_ms  = 0      # throttle print rate during LF

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
                self._io.write("\r\nREADY  f=start  c=calibrate\r\n" + UI_prompt)
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
                        elif ch in {"c", "C"}:
                            self._cal_enter()
                        else:
                            self._disarm()
                            self._io.write("\r\nSTOPPED  f=start  c=calibrate\r\n" + UI_prompt)

                        yield self._state
                        continue

            # ---- S4: line-follow timed run ----
            elif self._state == S4_LF:
                elapsed = ticks_diff(ticks_ms(), self._lf_start_ms)
                if self._io.any():
                    self._io.read1()   # consume key
                    self._disarm()
                    self._io.write("\r\nLF CANCELLED  f=start  c=calibrate\r\n" + UI_prompt)
                    self._state = S1_CMD
                    yield self._state
                    continue

                # Print centroid error vs time at ~50 ms intervals for plotting
                if self._line_err is not None:
                    now_ms = ticks_ms()
                    if ticks_diff(now_ms, self._lf_print_ms) >= 50:
                        self._lf_print_ms = now_ms
                        err = self._line_err.get()
                        t_s = elapsed / 1000.0
                        self._io.write("LF,{:.3f},{:.4f}\r\n".format(t_s, err))

                if elapsed >= LF_DURATION_MS:
                    self._disarm()
                    self._io.write("\r\nLF DONE (10 s)  f=start  c=calibrate\r\n" + UI_prompt)
                    self._state = S1_CMD

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
