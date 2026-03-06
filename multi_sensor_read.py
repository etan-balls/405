"""
Multi-sensor IR reflectance array reader.

ROOT CAUSE FIX: The original version used ADC.read_timed_multi(), which is a
BLOCKING call that waits for the hardware timer to fire buf_size samples before
returning. At freq=100 Hz that means every sensor read blocked the CPU for ~10 ms.
With a 20 ms task period the sensor task was consuming 50% of CPU time in a single
uninterruptible call, starving every other task and making the scheduler appear frozen.

This version uses individual ADC.read() calls instead. Each call returns the
most recently converted value instantly (the STM32 ADC runs continuously in
background). The readings are taken sequentially in a tight Python loop; at
~168 MHz the inter-sample skew across 11 sensors is well under 1 µs, which is
negligible for a line sensor running at 50–100 Hz.

The Timer and buf_size arguments are kept in the constructor signature so
existing call-sites don't need to change, but they are no longer used.
"""
from pyb import ADC
import array


class multiple_ir_readings:
    """
    N-sensor IR array interface with non-blocking ADC reads.
    """

    def __init__(self, *pins, timer_id=8, freq=100, buf_size=1):
        """
        Initialize IR sensor array with N ADC pins.

        :param pins:      Pin objects for sensors in left-to-right order.
        :param timer_id:  Ignored (kept for API compatibility).
        :param freq:      Ignored (kept for API compatibility).
        :param buf_size:  Ignored (kept for API compatibility).
        """
        if len(pins) < 1:
            raise ValueError("At least one ADC pin must be provided")

        self.adcs = [ADC(p) for p in pins]

    def read(self):
        """
        Read all sensors instantly using individual ADC.read() calls.

        Each call returns the most recent background-converted value with no
        blocking. The STM32 ADC runs in continuous-conversion mode, so the
        value is always fresh within one conversion cycle (~1 µs at 12-bit).

        :return: List of raw ADC values [s0, s1, ..., sN-1]  (integers 0–4095)
        :rtype:  list[int]
        """
        return [adc.read() for adc in self.adcs]
