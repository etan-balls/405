# task_bt.py
# HC-05 Bluetooth console task:
#   (1) ME405-style UI commands (mirrors task_user.py) over Bluetooth:
#         f = line follow timed run (10 s)
#         c = calibration mode
#             w = capture white
#             b = capture black
#             x = exit calibration
#         x / any key = cancel/stop during LF
#       It prints the same prompt token: ">: "
#
#   (2) Debug/telemetry + record-mode logger (line-based commands):
#         PING
#         GET
#         SET base <val>
#         SET kp_line <val>
#         SET kd_line <val>
#         SET max_effort <val>
#         SET hz <val>                (telemetry rate)
#         REC START <hz> <seconds>
#         REC START <hz> <samples> SAMPLES
#         REC STOP | REC DUMP | REC CLEAR | REC STATUS
#
# Notes:
# - IMU support is intentionally NOT implemented yet.
# - Encoder velocities are updated by calling encoder.update() before reads.

import pyb

UI_PROMPT = ">: "
N_SENSORS = 11

LF_DURATION_MS = 10_000
LF_PRINT_MS    = 50
CAL_PRINT_MS   = 200


class task_bt:
    def __init__(self, bt, *,
                 leftEnc, rightEnc,
                 line_sensor,
                 ctrl_obj,
                 # shares so we can mirror task_user behavior over BT
                 leftMotorGo=None, rightMotorGo=None,
                 lineFollowEnable=None,
                 armEnable=None,
                 lineError=None,
                 imu_heading=None,
                 imu_yawrate=None,
                 imu_calib=None,
                 baseEffort_share=None):

        self.bt = bt
        self.leftEnc = leftEnc
        self.rightEnc = rightEnc
        self.line = line_sensor
        self.ctrl = ctrl_obj

        self.leftMotorGo = leftMotorGo
        self.rightMotorGo = rightMotorGo
        self.lf_en = lineFollowEnable
        self.arm = armEnable
        self.line_err = lineError

        # IMU shares (optional)
        self.imu_heading = imu_heading
        self.imu_yawrate = imu_yawrate
        self.imu_calib = imu_calib

        self.baseEffort = baseEffort_share  # optional Share('f')

        # --------------------------
        # Live telemetry stream
        # --------------------------
        self._last_telem = pyb.millis()
        self.telem_period_ms = 200   # 5 Hz default (SET hz <n>)

        # --------------------------
        # Record mode (store to RAM, dump later)
        # --------------------------
        self.recording = False
        self.rec_period_ms = 20
        self.rec_target_samples = 0
        self.rec_samples = []
        self.rec_max_samples = 2000
        self._last_rec = pyb.millis()

        # --------------------------
        # UI mirror state (task_user-like)
        # --------------------------
        self._ui_state = "INIT"  # INIT, CMD, LF, CAL
        self._lf_start_ms = 0
        self._lf_last_print = 0
        self._cal_last_print = 0
        self._white_adc = None
        self._black_adc = None

    # --------------------------
    # Small output helpers
    # --------------------------
    def _writeln(self, s: str):
        self.bt.send_line(s)

    def _prompt(self):
        # Prompt is typically printed without extra text; we still send as its own line
        self.bt.send_line(UI_PROMPT.rstrip())

    # --------------------------
    # Hardware control helpers (mirrors task_user)
    # --------------------------
    def _disarm(self):
        if self.arm is not None:
            self.arm.put(False)
        if self.lf_en is not None:
            self.lf_en.put(False)
        if self.leftMotorGo is not None:
            self.leftMotorGo.put(False)
        if self.rightMotorGo is not None:
            self.rightMotorGo.put(False)

    def _lf_begin(self):
        if self.arm is not None:
            self.arm.put(True)
        if self.lf_en is not None:
            self.lf_en.put(True)
        if self.leftMotorGo is not None:
            self.leftMotorGo.put(True)
        if self.rightMotorGo is not None:
            self.rightMotorGo.put(True)
        self._lf_start_ms = pyb.millis()
        self._lf_last_print = self._lf_start_ms - LF_PRINT_MS
        self._writeln("LF ON (10 s)")
        self._ui_state = "LF"

    # --------------------------
    # Calibration helpers (mirrors task_user output style)
    # --------------------------
    def _cal_enter(self):
        self._disarm()
        self._white_adc = None
        self._black_adc = None
        self._cal_last_print = pyb.millis() - CAL_PRINT_MS
        self._writeln("")
        self._writeln("--- CALIBRATION MODE ---")
        self._writeln("w=capture white  b=capture black  x=exit")
        self._ui_state = "CAL"

    def _cal_print_live(self, readings):
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
                n = (r - white) / scale
                if n < 0.0:
                    n = 0.0
                elif n > 1.0:
                    n = 1.0
                num += w * n
                den += n
            error = (num / den) if den > 0.01 else 0.0

        w_str = str(self._white_adc) if self._white_adc is not None else "???"
        b_str = str(self._black_adc) if self._black_adc is not None else "???"
        adc_str = ",".join(str(int(r)) for r in readings)
        self._writeln("W={} B={} ERR={:+.3f} | {}".format(w_str, b_str, error, adc_str))

    def _cal_capture_white(self, readings):
        self._white_adc = int(sum(readings) / len(readings))
        self._writeln(">> WHITE captured: {}".format(self._white_adc))
        self._cal_try_recommend()

    def _cal_capture_black(self, readings):
        self._black_adc = int(max(readings))
        self._writeln(">> BLACK captured: {}".format(self._black_adc))
        self._cal_try_recommend()

    def _cal_try_recommend(self):
        if self._white_adc is None or self._black_adc is None:
            return
        w = self._white_adc
        b = self._black_adc
        scale = b - w
        contrast = abs(scale) / 4095.0 * 100.0

        self._writeln("")
        self._writeln("=== CALIBRATION RESULTS ===")
        self._writeln("white_adc = {}".format(w))
        self._writeln("black_adc = {}".format(b))
        qual = "(excellent)" if contrast > 40 else "(good)" if contrast > 20 else "(marginal - consider recalibrating)" if contrast > 10 else "(TOO LOW - recalibrate on better surface)"
        self._writeln("contrast  = {:.1f}%  {}".format(contrast, qual))

        self._writeln("")
        self._writeln("--- Recommended STEER_GAIN values ---")
        self._writeln("(based on max error = 5.0, adjust BASE_VEL column for your speed)")
        self._writeln("{:<12} {:<14} {}".format("STEER_GAIN", "Δvel @BASE400", "Behaviour"))
        self._writeln("-" * 44)

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
            pct = delta / 400 * 100
            rec = " <-- recommended" if 30 <= pct <= 60 else ""
            self._writeln("{:<12} {:<14} {}{}".format(str(k), "+/-{:.0f} ({:.0f}%)".format(delta, pct), label, rec))

        self._writeln("")
        self._writeln("Copy into main.py:")
        self._writeln("  black_adc  = {}".format(b))
        self._writeln("  white_adc  = {}".format(w))
        self._writeln("")
        self._writeln("Press x to exit calibration.")

    # --------------------------
    # Data snapshot (encoders + line + controller gains)
    # --------------------------
    def _read_snapshot(self):
        t = pyb.millis()

        # Refresh encoders so velocity is current
        try:
            self.leftEnc.update()
        except Exception:
            pass
        try:
            self.rightEnc.update()
        except Exception:
            pass

        el_pos = self.leftEnc.get_position()
        er_pos = self.rightEnc.get_position()
        el_vel = self.leftEnc.get_velocity()
        er_vel = self.rightEnc.get_velocity()

        line_err = self.line.calculate_error()
        raw = self.line.get_raw_readings()

        base = None
        if self.baseEffort is not None:
            try:
                base = self.baseEffort.get()
            except Exception:
                base = None

        kp_line = getattr(self.ctrl, "_kp_line", None)
        kd_line = getattr(self.ctrl, "_kd_line", None)
        max_eff = getattr(self.ctrl, "_max_effort", None)

        imu_h = None
        imu_w = None
        imu_c = None
        if self.imu_heading is not None:
            try:
                imu_h = self.imu_heading.get()
            except Exception:
                imu_h = None
        if self.imu_yawrate is not None:
            try:
                imu_w = self.imu_yawrate.get()
            except Exception:
                imu_w = None
        if self.imu_calib is not None:
            try:
                imu_c = self.imu_calib.get()
            except Exception:
                imu_c = None

        return (t, el_pos, er_pos, el_vel, er_vel, line_err, base, kp_line, kd_line, max_eff, raw, imu_h, imu_w, imu_c)

    # --------------------------
    # Live telemetry
    # --------------------------
    def send_telemetry(self):
        (t, el_pos, er_pos, el_vel, er_vel, line_err, base, kp_line, kd_line, max_eff, raw, imu_h, imu_w, imu_c) = self._read_snapshot()

        msg = (
            f"T t={t} "
            f"el_pos={el_pos} er_pos={er_pos} "
            f"el_vel={el_vel:.1f} er_vel={er_vel:.1f} "
            f"line_err={line_err:.3f} "
            f"base={base} kp_line={kp_line} kd_line={kd_line} max_eff={max_eff} "
            f"raw={raw} imu_h={imu_h} imu_w={imu_w} imu_cal={imu_c}"
        )
        self.bt.send_line(msg)

    # --------------------------
    # Record mode helpers
    # --------------------------
    def _rec_start(self, hz, seconds=None, samples=None):
        if hz <= 0:
            self.bt.send_line("ERR hz must be > 0")
            return

        self.rec_period_ms = max(1, int(1000 / hz))
        self.rec_samples = []

        if samples is not None:
            target = int(samples)
        else:
            target = int(hz * seconds)

        if target <= 0:
            self.bt.send_line("ERR target samples must be > 0")
            return

        if target > self.rec_max_samples:
            target = self.rec_max_samples
            self.bt.send_line(f"WARN capped to {self.rec_max_samples} samples")

        self.rec_target_samples = target
        self.recording = True
        self._last_rec = pyb.millis()
        self.bt.send_line(f"OK REC START hz={hz} period_ms={self.rec_period_ms} target={self.rec_target_samples}")

    def _rec_stop(self):
        self.recording = False
        self.bt.send_line(f"OK REC STOP n={len(self.rec_samples)}")

    def _rec_clear(self):
        self.recording = False
        self.rec_samples = []
        self.rec_target_samples = 0
        self.bt.send_line("OK REC CLEAR")

    def _rec_status(self):
        self.bt.send_line(
            f"REC status recording={int(self.recording)} "
            f"n={len(self.rec_samples)} target={self.rec_target_samples} "
            f"period_ms={self.rec_period_ms} max={self.rec_max_samples}"
        )

    def _rec_dump(self):
        self.bt.send_line("CSV t_ms,el_pos,er_pos,el_vel,er_vel,line_err,base,kp_line,kd_line,max_eff,raw,imu_heading,imu_yawrate,imu_calib")
        for s in self.rec_samples:
            (t, el_pos, er_pos, el_vel, er_vel, line_err, base, kp_line, kd_line, max_eff, raw, imu_h, imu_w, imu_c) = s
            self.bt.send_line(
                f"{t},{el_pos},{er_pos},{el_vel:.3f},{er_vel:.3f},{line_err:.6f},"
                f"{base},{kp_line},{kd_line},{max_eff},{raw},{imu_h},{imu_w},{imu_c}"
            )
        self.bt.send_line(f"OK REC DUMP n={len(self.rec_samples)}")

    # --------------------------
    # Parsing commands
    # --------------------------
    def _handle_ui_char(self, ch: str):
        ch = ch.strip()
        if not ch:
            return

        # State-specific behavior (matches task_user)
        if self._ui_state in ("INIT", "CMD"):
            if ch.lower() == "f":
                self._lf_begin()
            elif ch.lower() == "c":
                self._cal_enter()
            else:
                self._disarm()
                self._writeln("STOPPED  f=start  c=calibrate")
                self._prompt()
                self._ui_state = "CMD"
            return

        if self._ui_state == "LF":
            # any key cancels
            self._disarm()
            self._writeln("LF CANCELLED  f=start  c=calibrate")
            self._prompt()
            self._ui_state = "CMD"
            return

        if self._ui_state == "CAL":
            readings = self.line.get_raw_readings()
            if ch.lower() == "w":
                self._cal_capture_white(readings)
            elif ch.lower() == "b":
                self._cal_capture_black(readings)
            elif ch.lower() in ("x", "q"):
                self._writeln("Exiting calibration.")
                self._prompt()
                self._ui_state = "CMD"
            # else ignore
            return

    def _handle_line_cmd(self, line: str) -> bool:
        """
        Returns True if line was consumed as a debug/record command.
        """
        parts = line.split()
        if not parts:
            return False

        cmd = parts[0].upper()

        if cmd == "PING":
            self.bt.send_line("PONG")
            return True

        if cmd == "GET":
            self.send_telemetry()
            return True

        if cmd == "SET" and len(parts) == 3:
            key = parts[1].lower()
            try:
                val = float(parts[2])
            except Exception:
                self.bt.send_line("ERR value must be numeric")
                return True

            if key == "base":
                if self.baseEffort is None:
                    self.bt.send_line("ERR baseEffort share not wired")
                    return True
                self.baseEffort.put(val)
            elif key == "kp_line":
                setattr(self.ctrl, "_kp_line", val)
            elif key == "kd_line":
                setattr(self.ctrl, "_kd_line", val)
            elif key == "max_effort":
                setattr(self.ctrl, "_max_effort", val)
            elif key == "yawrate_gain":
                # IMU gyro damping gain
                try:
                    self.ctrl.set_imu_gains(yawrate_gain=val, heading_gain=getattr(self.ctrl, "_k_heading", 0.0))
                except Exception:
                    pass
            elif key == "heading_gain":
                # IMU heading hold gain
                try:
                    self.ctrl.set_imu_gains(yawrate_gain=getattr(self.ctrl, "_k_yawrate", 0.0), heading_gain=val)
                except Exception:
                    pass
            elif key == "hz":
                if val > 0:
                    self.telem_period_ms = int(1000 / val)
            else:
                self.bt.send_line("ERR unknown key")
                return True

            self.bt.send_line("OK")
            return True

        if cmd == "REC":
            if len(parts) < 2:
                self.bt.send_line("ERR usage: REC <START|STOP|DUMP|CLEAR|STATUS> ...")
                return True

            sub = parts[1].upper()

            if sub == "START":
                if len(parts) < 4:
                    self.bt.send_line("ERR usage: REC START <hz> <seconds>  OR  REC START <hz> <samples> SAMPLES")
                    return True
                hz = float(parts[2])
                third = parts[3]
                if len(parts) >= 5 and parts[4].upper() == "SAMPLES":
                    self._rec_start(hz, samples=int(third))
                else:
                    self._rec_start(hz, seconds=float(third))
                return True

            if sub == "STOP":
                self._rec_stop()
                return True

            if sub == "DUMP":
                self._rec_dump()
                return True

            if sub == "CLEAR":
                self._rec_clear()
                return True

            if sub == "STATUS":
                self._rec_status()
                return True

            self.bt.send_line("ERR unknown REC subcmd")
            return True

        return False

    # --------------------------
    # Scheduler tasks
    # --------------------------
    def run_poll(self):
        """Fast poller for UART RX (line assembly)."""
        while True:
            self.bt.poll()
            yield 0

    def run_app(self):
        """
        - Mirrors task_user UI (f/c/w/b/x) over Bluetooth
        - Parses debug/record commands
        - Records samples into RAM if REC mode enabled
        - Streams live telemetry at a lower rate
        """
        while True:
            now = pyb.millis()

            # ---- UI init ----
            if self._ui_state == "INIT":
                self._disarm()
                self._writeln("READY  f=start  c=calibrate")
                self._prompt()
                self._ui_state = "CMD"

            # ---- Incoming commands (line-based) ----
            if self.bt.any_line():
                line = self.bt.get_line()
                if line:
                    # 1) If it matches debug/record commands, consume there
                    if self._handle_line_cmd(line):
                        pass
                    else:
                        # 2) Otherwise treat as UI char(s): first non-space char
                        ch = line.strip()[:1]
                        if ch:
                            self._handle_ui_char(ch)

            # ---- UI timed behaviors ----
            if self._ui_state == "LF":
                elapsed = pyb.elapsed_millis(self._lf_start_ms)
                # Print LF,t,err for plotting at ~50 ms intervals
                if self.line_err is not None and pyb.elapsed_millis(self._lf_last_print) >= LF_PRINT_MS:
                    self._lf_last_print = now
                    try:
                        err = self.line_err.get()
                    except Exception:
                        err = 0.0
                    t_s = elapsed / 1000.0
                    imu_h = None
                    imu_w = None
                    if self.imu_heading is not None:
                        try:
                            imu_h = self.imu_heading.get()
                        except Exception:
                            imu_h = None
                    if self.imu_yawrate is not None:
                        try:
                            imu_w = self.imu_yawrate.get()
                        except Exception:
                            imu_w = None

                    self._writeln("LF,{:.3f},{:.4f},{},{}".format(t_s, err, imu_h, imu_w))

                if elapsed >= LF_DURATION_MS:
                    self._disarm()
                    self._writeln("LF DONE (10 s)  f=start  c=calibrate")
                    self._prompt()
                    self._ui_state = "CMD"

            elif self._ui_state == "CAL":
                if pyb.elapsed_millis(self._cal_last_print) >= CAL_PRINT_MS:
                    self._cal_last_print = now
                    readings = self.line.get_raw_readings()
                    self._cal_print_live(readings)

            # ---- Record sampling (independent) ----
            if self.recording and pyb.elapsed_millis(self._last_rec) >= self.rec_period_ms:
                self._last_rec = now
                if len(self.rec_samples) < self.rec_target_samples:
                    self.rec_samples.append(self._read_snapshot())
                else:
                    self._rec_stop()

            # ---- Live telemetry (independent) ----
            if pyb.elapsed_millis(self._last_telem) >= self.telem_period_ms:
                self._last_telem = now
                self.send_telemetry()

            yield 0
