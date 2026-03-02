# bt_console.py
# HC-05 Bluetooth "serial port" console helper for MicroPython (pyb.UART).
#
# Usage:
#   from bt_console import BTConsole
#   bt = BTConsole(uart_num=2, baud=9600)
#   bt.poll()
#   if bt.any_line(): cmd = bt.get_line()
#   bt.send_line("hello")

import pyb


class BTConsole:
    """
    HC-05 over UART for:
      - non-blocking receive of newline-delimited commands
      - line-based transmit for telemetry/debug
    """

    def __init__(self, uart_num=2, baud=9600, echo=False):
        # Common ME405 mapping: UART(2) -> USART2 on PA2/PA3 (Arduino D1/D0)
        self.uart = pyb.UART(uart_num, baudrate=baud, bits=8, parity=None, stop=1)
        self.echo = echo
        self._buf = bytearray()
        self._lines = []

    def poll(self):
        """Call often from a fast task. Non-blocking."""
        n = self.uart.any()
        if not n:
            return

        data = self.uart.read(n) or b""
        if self.echo and data:
            self.uart.write(data)

        self._buf.extend(data)

        # Split into complete lines
        while True:
            try:
                i = self._buf.index(10)  # '\n'
            except ValueError:
                break

            raw = self._buf[:i]
            del self._buf[:i + 1]

            # strip '\r'
            if raw.endswith(b'\r'):
                raw = raw[:-1]

            try:
                line = raw.decode("utf-8").strip()
            except Exception:
                line = ""

            if line:
                self._lines.append(line)

    def any_line(self):
        return len(self._lines) > 0

    def get_line(self):
        if self._lines:
            return self._lines.pop(0)
        return None

    def send_line(self, s):
        try:
            self.uart.write(str(s))
            self.uart.write("\r\n")
        except Exception:
            pass
