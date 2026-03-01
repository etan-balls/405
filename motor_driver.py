import pyb
from pyb import Pin, Timer

class motor_driver:
    """Motor driver for DRV8833-style H-bridge.

    set_effort(effort):
        effort > 0  -> forward  (DIR high, PWM = effort%)
        effort < 0  -> reverse  (DIR low,  PWM = abs(effort)%)
        effort == 0 -> brake    (PWM = 0, direction unchanged)

    FIX #4 (direction / zero-effort):
      The original code used ``if effort > 0`` which sent effort=0 into the
      ``else`` branch, setting DIR low before braking. While usually harmless,
      it caused an unexpected direction latch on the next non-zero command.
      The rewrite uses an explicit three-way split so zero is handled cleanly
      and direction is only changed when effort is actually non-zero.
    """

    def __init__(self, nSLP, DIR, PWM, tim, chan):
        """Initializes a Motor object."""
        self.nSLP_pin = Pin(nSLP, Pin.OUT_PP)
        self.DIR_pin  = Pin(DIR,  Pin.OUT_PP)
        self.PWM_pin  = Pin(PWM)
        self.tim_pin  = tim
        self.pwm_ch   = self.tim_pin.channel(chan, Timer.PWM, pin=self.PWM_pin)

    def set_effort(self, effort):
        """Set motor effort in the range [-100, 100].

        Positive values drive forward, negative drive reverse, zero brakes.
        """
        if effort > 0:
            self.DIR_pin.high()
            self.pwm_ch.pulse_width_percent(min(effort, 100))
        elif effort < 0:
            self.DIR_pin.low()
            self.pwm_ch.pulse_width_percent(min(-effort, 100))
        else:
            # effort == 0: brake (hold DIR, kill PWM)
            self.pwm_ch.pulse_width_percent(0)

    def enable(self):
        """Wake the driver and hold PWM at 0 until a command arrives."""
        self.pwm_ch.pulse_width_percent(0)
        self.nSLP_pin.high()

    def disable(self):
        """Put the driver to sleep (low-power brake)."""
        self.pwm_ch.pulse_width_percent(0)
        self.nSLP_pin.low()
