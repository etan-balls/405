import argparse
import os
import re
import sys
import time
import threading
from datetime import datetime

import serial

# Optional plotting
try:
    import matplotlib.pyplot as plt  # type: ignore
    HAS_MPL = True
except Exception:
    HAS_MPL = False

# Windows key input
if os.name == "nt":
    import msvcrt
else:
    msvcrt = None  # Windows-first; fallback is line-based input


NUMERIC_TV_RE = re.compile(
    r"^\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*$"
)

LF_RE = re.compile(
    r"^\s*LF\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)\s*,\s*([+-]?\d*\.?\d+(?:[eE][+-]?\d+)?)"
    r"(?:\s*,\s*([^,\s]+))?(?:\s*,\s*([^,\s]+))?\s*$",
    re.IGNORECASE
)


def ts_base() -> str:
    return datetime.now().strftime("%m_%d_%H_%M_%S")


def ensure_log_dir(outdir: str) -> str:
    log_dir = os.path.join(outdir, "Collection Log")
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


def safe_print(s: str) -> None:
    sys.stdout.write(s)
    sys.stdout.flush()


class BaseCapture:
    """
    Base capture that:
      - stores raw serial lines
      - stops when data started AND prompt token appears again, or via timeout
    """
    def __init__(self, log_dir: str, base_name: str, prompt_token: str = ">:"):
        self.log_dir = log_dir
        self.base_name = base_name
        self.prompt_token = prompt_token

        self.active = False
        self.finished = False
        self.data_started = False

        self.raw_lines: list[str] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        with self._lock:
            self.active = True
            self.finished = False
            self.data_started = False
            self.raw_lines = []

    def feed_line(self, line: str) -> None:
        with self._lock:
            if not self.active or self.finished:
                return

            self.raw_lines.append(line)

            # Stop condition: prompt appears after data has started
            if self.data_started and line.strip().endswith(self.prompt_token):
                self.finished = True
                self.active = False
                return

            self._parse_line(line)

    def _parse_line(self, line: str) -> None:
        raise NotImplementedError

    def is_done(self) -> bool:
        with self._lock:
            return self.finished

    def stop_due_to_timeout(self, reason: str) -> None:
        with self._lock:
            if self.finished:
                return
            self.raw_lines.append(f"[WARN] Capture ended: {reason}")
            self.finished = True
            self.active = False

    def save_all(self) -> list[str]:
        raise NotImplementedError


class CaptureG(BaseCapture):
    """
    Capture one dual-motor 'g' run.
    Romi prints:
      - "Left motor response"
      - time,value lines
      - "Right motor response"
      - time,value lines
    """
    def __init__(self, log_dir: str, base_name: str, prompt_token: str = ">:"):
        super().__init__(log_dir, base_name, prompt_token)
        self.current_section = None  # "left" | "right" | None
        self.left: list[tuple[float, float]] = []
        self.right: list[tuple[float, float]] = []

    def start(self) -> None:
        super().start()
        with self._lock:
            self.current_section = None
            self.left = []
            self.right = []

    def _parse_line(self, line: str) -> None:
        low = line.lower()
        if "left motor response" in low:
            self.current_section = "left"
            return
        if "right motor response" in low:
            self.current_section = "right"
            return

        m = NUMERIC_TV_RE.match(line)
        if m and self.current_section in ("left", "right"):
            t = float(m.group(1))
            v = float(m.group(2))
            self.data_started = True
            if self.current_section == "left":
                self.left.append((t, v))
            else:
                self.right.append((t, v))

    def save_all(self) -> list[str]:
        saved: list[str] = []
        base = self.base_name

        # Raw .txt
        txt_path = os.path.join(self.log_dir, f"{base}.txt")
        with open(txt_path, "w", encoding="utf-8", newline="") as f:
            for ln in self.raw_lines:
                f.write(ln + "\n")
        saved.append(txt_path)

        # CSVs
        left_csv = os.path.join(self.log_dir, f"{base}_left.csv")
        right_csv = os.path.join(self.log_dir, f"{base}_right.csv")

        with open(left_csv, "w", encoding="utf-8", newline="") as f:
            f.write("time,value\n")
            for t, v in self.left:
                f.write(f"{t},{v}\n")
        saved.append(left_csv)

        with open(right_csv, "w", encoding="utf-8", newline="") as f:
            f.write("time,value\n")
            for t, v in self.right:
                f.write(f"{t},{v}\n")
        saved.append(right_csv)

        # Plots (optional)
        if not HAS_MPL:
            safe_print("[WARN] matplotlib not installed; skipping PNG plots.\n")
            safe_print("       Install with: py -m pip install matplotlib\n")
            return saved

        def plot_one(series: list[tuple[float, float]], title: str, out_path: str) -> None:
            plt.figure()
            if series:
                xs = [p[0] for p in series]
                ys = [p[1] for p in series]
                plt.plot(xs, ys)
            plt.title(title)
            plt.xlabel("time")
            plt.ylabel("value")
            plt.tight_layout()
            plt.savefig(out_path)
            plt.close()

        left_png = os.path.join(self.log_dir, f"{base}_left.png")
        right_png = os.path.join(self.log_dir, f"{base}_right.png")

        plot_one(self.left, "Left motor response", left_png)
        saved.append(left_png)

        plot_one(self.right, "Right motor response", right_png)
        saved.append(right_png)

        return saved


class CaptureF(BaseCapture):
    """
    Capture one line-follow 'f' run.
    Expected lines:
      LF,t,err
    Stops when prompt returns (after data started).
    """
    def __init__(self, log_dir: str, base_name: str, prompt_token: str = ">:"):
        super().__init__(log_dir, base_name, prompt_token)
        self.series: list[tuple[float, float, str | None, str | None]] = []

    def start(self) -> None:
        super().start()
        with self._lock:
            self.series = []

    def _parse_line(self, line: str) -> None:
        m = LF_RE.match(line)
        if m:
            t = float(m.group(1))
            err = float(m.group(2))
            imu_h = m.group(3)
            imu_w = m.group(4)
            if imu_h is not None and imu_h.lower() == 'none':
                imu_h = None
            if imu_w is not None and imu_w.lower() == 'none':
                imu_w = None
            self.data_started = True
            self.series.append((t, err, imu_h, imu_w))

    def save_all(self) -> list[str]:
        saved: list[str] = []
        base = self.base_name

        # Raw .txt
        txt_path = os.path.join(self.log_dir, f"{base}.txt")
        with open(txt_path, "w", encoding="utf-8", newline="") as f:
            for ln in self.raw_lines:
                f.write(ln + "\n")
        saved.append(txt_path)

        # CSV
        csv_path = os.path.join(self.log_dir, f"{base}.csv")
        with open(csv_path, "w", encoding="utf-8", newline="") as f:
            f.write("time,err,imu_heading_deg,imu_yawrate_dps\n")
            for t, e, h, w in self.series:
                f.write(f"{t},{e},{h},{w}\n")
        saved.append(csv_path)

        # Plot (optional)
        if not HAS_MPL:
            safe_print("[WARN] matplotlib not installed; skipping PNG plots.\n")
            safe_print("       Install with: py -m pip install matplotlib\n")
            return saved

        png_path = os.path.join(self.log_dir, f"{base}.png")
        plt.figure()
        if self.series:
            xs = [p[0] for p in self.series]
            ys = [p[1] for p in self.series]
            plt.plot(xs, ys)
        plt.title("Line-follow error (LF)")
        plt.xlabel("time (s)")
        plt.ylabel("error")
        plt.tight_layout()
        plt.savefig(png_path)
        plt.close()
        saved.append(png_path)
        return saved


class SerialTerminal:
    """
    PuTTY-like terminal:
      - Reader thread prints Romi output live
      - Main thread forwards keystrokes to Romi

    Auto-capture triggers:
      - If user types 'g' + Enter -> CaptureG
      - If user types 'f' + Enter -> CaptureF
    Auto mode:
      - --auto repeatedly sends --cmd (g or f) with intervals
    """
    def __init__(self, ser: serial.Serial, log_dir: str, prompt_token: str = ">:"):
        self.ser = ser
        self.log_dir = log_dir
        self.prompt_token = prompt_token

        self.stop_event = threading.Event()
        self.capture_lock = threading.Lock()
        self.capture: BaseCapture | None = None

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._tx_line_buf: list[str] = []

    def start(self) -> None:
        self._reader_thread.start()

    def close(self) -> None:
        self.stop_event.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def _reader_loop(self) -> None:
        buf = bytearray()
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(256)
            except Exception as e:
                safe_print(f"\n[ERR] Serial read failed: {e}\n")
                self.stop_event.set()
                return

            if not chunk:
                continue

            safe_print(chunk.decode("utf-8", errors="replace"))

            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl == -1:
                    break
                raw_line = buf[:nl]
                del buf[:nl + 1]
                raw_line = raw_line.rstrip(b"\r")
                line = raw_line.decode("utf-8", errors="replace")

                with self.capture_lock:
                    cap = self.capture
                if cap is not None:
                    cap.feed_line(line)

    def _write_bytes(self, b: bytes) -> None:
        try:
            self.ser.write(b)
        except Exception as e:
            safe_print(f"\n[ERR] Serial write failed: {e}\n")
            self.stop_event.set()

    def _begin_capture(self, kind: str) -> BaseCapture:
        base = ts_base()
        if kind.lower() == "f":
            cap: BaseCapture = CaptureF(self.log_dir, base_name=base, prompt_token=self.prompt_token)
        else:
            cap = CaptureG(self.log_dir, base_name=base, prompt_token=self.prompt_token)
        cap.start()
        with self.capture_lock:
            self.capture = cap
        return cap

    def _end_capture_and_save(self, cap: BaseCapture, reason: str | None = None) -> None:
        if reason:
            cap.stop_due_to_timeout(reason)

        saved = cap.save_all()
        for p in saved:
            safe_print(f"\n[SAVED] {p}\n")

        with self.capture_lock:
            if self.capture is cap:
                self.capture = None

    def _wait_for_capture_and_save(
        self,
        cap: BaseCapture,
        start_timeout_s: float,
        overall_timeout_s: float,
    ) -> None:
        t0 = time.time()

        while not self.stop_event.is_set():
            if cap.is_done():
                self._end_capture_and_save(cap)
                return

            with cap._lock:
                started = cap.data_started

            if not started and (time.time() - t0) > start_timeout_s:
                self._end_capture_and_save(cap, reason="timeout waiting for data start")
                return

            if (time.time() - t0) > overall_timeout_s:
                self._end_capture_and_save(cap, reason="overall timeout waiting for prompt return")
                return

            time.sleep(0.02)

    def run_interactive(self, start_timeout_s: float, overall_timeout_s: float) -> None:
        safe_print("[INFO] Interactive terminal mode.\n")
        safe_print("[INFO] Type as usual. When you run 'g' or 'f', capture/export happens automatically.\n")
        safe_print("[INFO] Press Ctrl+C to exit.\n\n")

        try:
            while not self.stop_event.is_set():
                if os.name == "nt" and msvcrt is not None:
                    if msvcrt.kbhit():
                        ch = msvcrt.getwch()

                        if ch == "\x03":
                            raise KeyboardInterrupt

                        if ch in ("\b", "\x7f"):
                            if self._tx_line_buf:
                                self._tx_line_buf.pop()
                            self._write_bytes(b"\b \b")
                            continue

                        if ch in ("\r", "\n"):
                            line = "".join(self._tx_line_buf).strip()
                            self._tx_line_buf = []
                            self._write_bytes(b"\r\n")

                            if line.lower() in ("g", "f"):
                                cap = self._begin_capture(line.lower())
                                safe_print(f"\n[INFO] Capture started ({cap.base_name}) kind={line.lower()}\n")
                                self._wait_for_capture_and_save(cap, start_timeout_s, overall_timeout_s)
                            continue

                        self._tx_line_buf.append(ch)
                        self._write_bytes(ch.encode("utf-8", errors="replace"))
                    else:
                        time.sleep(0.01)
                else:
                    # fallback
                    line = sys.stdin.readline()
                    if not line:
                        time.sleep(0.05)
                        continue
                    self._write_bytes(line.encode("utf-8", errors="replace"))
                    if line.strip().lower() in ("g", "f"):
                        kind = line.strip().lower()
                        cap = self._begin_capture(kind)
                        safe_print(f"\n[INFO] Capture started ({cap.base_name}) kind={kind}\n")
                        self._wait_for_capture_and_save(cap, start_timeout_s, overall_timeout_s)

        except KeyboardInterrupt:
            safe_print("\n[INFO] Exiting...\n")
            self.stop_event.set()

    def run_auto(
        self,
        cmd: str,
        count: int,
        interval_s: float,
        prewait_s: float,
        start_timeout_s: float,
        overall_timeout_s: float,
    ) -> None:
        cmd = cmd.lower().strip()
        if cmd not in ("g", "f"):
            safe_print("[ERR] --cmd must be 'g' or 'f'\n")
            return

        safe_print("[INFO] Auto mode enabled.\n")
        safe_print(f"[INFO] cmd={cmd} prewait={prewait_s}s count={count} interval={interval_s}s\n")
        time.sleep(max(0.0, prewait_s))

        for i in range(count):
            if self.stop_event.is_set():
                break

            safe_print(f"\n[INFO] Auto run {i+1}/{count}: sending '{cmd}'\n")
            cap = self._begin_capture(cmd)
            safe_print(f"[INFO] Capture started ({cap.base_name}) kind={cmd}\n")

            self._write_bytes((cmd + "\r\n").encode("utf-8"))
            self._wait_for_capture_and_save(cap, start_timeout_s, overall_timeout_s)

            if i < count - 1 and not self.stop_event.is_set():
                safe_print(f"[INFO] Waiting {interval_s}s until next run...\n")
                time.sleep(max(0.0, interval_s))

        safe_print("\n[INFO] Auto mode complete. Returning to interactive terminal.\n")
        self.run_interactive(start_timeout_s, overall_timeout_s)


def main() -> None:
    ap = argparse.ArgumentParser(description="PuTTY-like terminal + automated capture/export for Romi (ME405).")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM9")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--outdir", default=None, help=r"Output directory, e.g. C:\fake_putty (default: script folder)")
    ap.add_argument("--auto", action="store_true", help="Enable fully automated batch captures")
    ap.add_argument("--cmd", default="g", help="Command to run in auto mode: g (step) or f (line follow). Default g")
    ap.add_argument("--count", type=int, default=1, help="Number of captures in auto mode")
    ap.add_argument("--interval", type=float, default=10.0, help="Seconds between captures in auto mode")
    ap.add_argument("--prewait", type=float, default=0.0, help="Seconds to wait before first auto capture")
    ap.add_argument("--prompt", default=">:", help="Prompt token indicating UI ready again (default '>:')")
    ap.add_argument("--start-timeout", type=float, default=5.0, help="Timeout waiting for first data (seconds)")
    ap.add_argument("--overall-timeout", type=float, default=40.0, help="Timeout waiting for prompt return (seconds)")
    args = ap.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    outdir = args.outdir if args.outdir else script_dir
    log_dir = ensure_log_dir(outdir)

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            write_timeout=1.0,
        )
    except serial.SerialException as e:
        msg = str(e).lower()
        if "access is denied" in msg or "permission" in msg or "cannot open" in msg:
            safe_print(f"[ERR] Could not open {args.port}. Is PuTTY/another program using it?\n")
            safe_print(f"      Details: {e}\n")
        else:
            safe_print(f"[ERR] Serial error opening {args.port}: {e}\n")
        return

    safe_print(f"[OK] Connected on {args.port} @ {args.baud}\n")
    safe_print(f"[INFO] Logs will be saved to: {log_dir}\n")
    if not HAS_MPL:
        safe_print("[WARN] matplotlib not installed; PNG plots will be skipped.\n")
        safe_print("       Install with: py -m pip install matplotlib\n")

    term = SerialTerminal(ser, log_dir=log_dir, prompt_token=args.prompt)
    term.start()

    if args.auto:
        term.run_auto(
            cmd=args.cmd,
            count=max(1, args.count),
            interval_s=max(0.0, args.interval),
            prewait_s=max(0.0, args.prewait),
            start_timeout_s=max(0.1, args.start_timeout),
            overall_timeout_s=max(0.1, args.overall_timeout),
        )
    else:
        term.run_interactive(
            start_timeout_s=max(0.1, args.start_timeout),
            overall_timeout_s=max(0.1, args.overall_timeout),
        )

    term.close()


if __name__ == "__main__":
    main()
