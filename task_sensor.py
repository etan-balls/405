import micropython
from task_share import Share

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_line_sensor:
    """Periodic line sensor task.

    Publishes line error to a Share('f').

    Only reads the sensor hardware when lineFollowEnable is True (or when no
    enable share is provided). This avoids burning CPU on ADC reads when the
    line follower is idle, and prevents stale error values from accumulating.
    """

    def __init__(self, line_sensor, line_error_share: Share, enable_share: Share = None):
        self._state = S0_INIT
        self._line  = line_sensor
        self._err   = line_error_share
        self._en    = enable_share
        print("Sensor Task instantiated")

    def run(self):
        while True:
            if self._state == S0_INIT:
                self._state = S1_RUN

            elif self._state == S1_RUN:
                # Always sample and publish so task_user can read live error
                # during line follow. Publishing 0.0 when not following is fine
                # because task_control checks lineFollowEnable itself.
                e = self._line.calculate_error()
                if e is None:
                    e = 0.0
                self._err.put(float(e))

            yield self._state
