# task_motor.py
from motor_driver import motor_driver
from task_share import Share
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_motor:
    """
    Motor actuation task.

    Safety additions:
      - armEnable Share: motors will ONLY run if armEnable.get() is True AND goFlag is True.
      - effort_cmd Share is assumed to contain float; if None, treated as 0.

    Direction:
      - invert=True flips effort sign in software.

    FIX: _safe_off() was previously called every single tick while in S1_WAIT.
    This meant that even after arm+go flags were set by the user task, the very
    next scheduler tick would call disable() again before the motor could start,
    making it impossible to restart after the first run. _safe_off() is now only
    called ONCE on entry to S1_WAIT (i.e. when transitioning INTO it from S0 or
    S2), not on every subsequent tick.
    """

    def __init__(self, mot: motor_driver, goFlag: Share, effort_cmd: Share, armEnable: Share, invert: bool = False):
        self._state = S0_INIT
        self._mot = mot
        self._goFlag = goFlag
        self._effort_cmd = effort_cmd
        self._arm = armEnable
        self._invert = bool(invert)
        print("Motor Task instantiated")

    def _safe_off(self):
        try:
            self._mot.set_effort(0)
        except Exception:
            pass
        try:
            self._mot.disable()
        except Exception:
            pass

    def run(self):
        while True:
            if self._state == S0_INIT:
                # Disable once at startup then wait
                self._safe_off()
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                # Do NOT call _safe_off() here every tick — that would
                # continuously disable the hardware and prevent restart.
                # Hardware was already shut down when we entered this state
                # (either from S0_INIT above or from S2_RUN below).

                # Transition to running as soon as both flags are set
                if self._arm.get() and self._goFlag.get():
                    self._mot.enable()
                    self._state = S2_RUN

            elif self._state == S2_RUN:
                # If disarmed or stop requested, shut down and go back to wait
                if (not self._arm.get()) or (not self._goFlag.get()):
                    self._safe_off()          # shut down once on exit
                    self._state = S1_WAIT
                    yield self._state
                    continue

                cmd = self._effort_cmd.get()
                if cmd is None:
                    cmd = 0.0
                if self._invert:
                    cmd = -cmd
                self._mot.set_effort(cmd)

            yield self._state
