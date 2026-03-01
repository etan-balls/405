import pyb
from time import ticks_us, ticks_diff

class encoder:
    """Quadrature encoder driver using STM32 timer encoder mode.

    IMPORTANT FIX:
      The original version returned velocity in counts per *microsecond* (counts/us),
      which makes typical velocities ~0.001..0.02 and causes controllers with
      setpoints like 3000 to saturate at max effort.

      This version returns velocity in counts per *second* (counts/s):
          velocity = delta / dt_us * 1_000_000
    """
    def __init__(self, tim, chA, chB):
        self.tim_pin = pyb.Timer(tim, period=0xFFFF, prescaler=0)
        self.tim_pin.channel(1, pin=chA, mode=self.tim_pin.ENC_AB)
        self.tim_pin.channel(2, pin=chB, mode=self.tim_pin.ENC_AB)

        self.position   = 0
        self.prev_count = self.tim_pin.counter()
        self.delta      = 0
        self.dt         = 0
        self.prev_time  = ticks_us()
        self.velocity   = 0.0

    def update(self):
        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time)   # dt in microseconds
        self.prev_time = now

        count = self.tim_pin.counter()
        self.delta = count - self.prev_count

        # wraparound handling
        if abs(self.delta) >= 60000:
            if self.delta < 0:
                self.delta += 65536
            else:
                self.delta -= 65536

        self.position += self.delta
        self.prev_count = count

        if self.dt > 0:
            # counts per second
            self.velocity = (self.delta / self.dt) * 1_000_000.0
        else:
            self.velocity = 0.0

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity

    def zero(self):
        self.position = 0
        self.prev_count = self.tim_pin.counter()
        self.prev_time = ticks_us()
        self.velocity = 0.0
