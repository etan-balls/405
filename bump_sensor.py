import pyb
from pyb import Pin


class BumpSensors:
    """
    Simple driver for the ME405 bumper boards.

    This driver assumes the bumper boards are wired as *active‑low* digital inputs
    with pull‑ups enabled on the Nucleo:

      Left bumper board:
        - GND  -> CN7-20  (board ground)
        - BUMP1 -> PB12
        - BUMP2 -> PB11
        - BUMP3 -> PC7

      Right bumper board:
        - BUMP1 -> PA15
        - BUMP2 -> PH0
        - BUMP3 -> PH1

    Pressing a bumper pulls the corresponding pin LOW (0); this class converts
    the raw pin levels into convenient booleans where True = "pressed".
    """

    def __init__(
        self,
        left_pins=None,
        right_pins=None,
        active_low: bool = True,
    ):
        """
        Create a bumper sensor interface.

        :param left_pins:  Iterable of Pin.cpu.* objects for left bumper inputs.
                           If None, defaults to PB12, PB11, PC7.
        :param right_pins: Iterable of Pin.cpu.* objects for right bumper inputs.
                           If None, defaults to PA15, PH0, PH1.
        :param active_low: If True (default), interpret 0 as pressed.
        """
        if left_pins is None:
            left_pins = [Pin.cpu.B12, Pin.cpu.B11, Pin.cpu.C7]
        if right_pins is None:
            right_pins = [Pin.cpu.A15, Pin.cpu.H0, Pin.cpu.H1]

        self._active_low = bool(active_low)

        # Configure all pins as inputs with internal pull‑ups as specified
        self._left = [Pin(p, Pin.IN, Pin.PULL_UP) for p in left_pins]
        self._right = [Pin(p, Pin.IN, Pin.PULL_UP) for p in right_pins]

    # ------------------------------------------------------------------
    # Raw readings
    # ------------------------------------------------------------------
    def _pin_to_pressed(self, pin: Pin) -> bool:
        """Convert a Pin object level to a pressed boolean."""
        val = pin.value()
        if self._active_low:
            return val == 0
        return val != 0

    def read_left(self):
        """
        Read all left bumper inputs.

        :return: List[bool] of length 3, True where that bumper is pressed.
        """
        return [self._pin_to_pressed(p) for p in self._left]

    def read_right(self):
        """
        Read all right bumper inputs.

        :return: List[bool] of length 3, True where that bumper is pressed.
        """
        return [self._pin_to_pressed(p) for p in self._right]

    def read_all(self):
        """
        Read all bumpers on both boards.

        :return: (left_list, right_list), each a List[bool] of length 3.
        """
        return self.read_left(), self.read_right()

    # ------------------------------------------------------------------
    # Convenience predicates
    # ------------------------------------------------------------------
    def any_left(self) -> bool:
        """Return True if any left bumper is pressed."""
        return any(self.read_left())

    def any_right(self) -> bool:
        """Return True if any right bumper is pressed."""
        return any(self.read_right())

    def any(self) -> bool:
        """Return True if any bumper (left or right) is pressed."""
        l, r = self.read_all()
        return any(l) or any(r)

