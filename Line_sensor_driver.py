from pyb import Pin, ADC, Timer  # kept for compatibility with existing project imports


class L_sensor:
    """
    Line following controller with weighted centroid error calculation.

    This driver supports arbitrary numbers of analog line sensors (e.g., Pololu QTR/QTRX arrays).
    It computes a normalized reflectance for each sensor and then returns a weighted-centroid error.

    Typical weights:
      - 7 sensors  -> [-3, -2, -1, 0, 1, 2, 3]
      - 11 sensors -> [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5]
    """

    def __init__(self, multi_sensor_read_object, black, white, bias=0.0, weights=None, sensor_count=None):
        """
        Initialize line follower with calibration values and steering bias.

        :param multi_sensor_read_object: IR sensor array object with a .read() -> list[int]
        :param black: ADC value for black surface (line). May be a single
                      scalar for all sensors or a list/tuple of per-sensor
                      values.
        :param white: ADC value for white surface (background). May be a single
                      scalar for all sensors or a list/tuple of per-sensor
                      values.
        :param bias: Steering bias (positive=right, negative=left)
        :param weights: Optional list of weights (same length as sensor readings). If None,
                        symmetric weights are generated automatically from sensor_count.
        :param sensor_count: Optional count of sensors. If None and weights is None, we will
                             take one read() during init to infer the count.
        """
        self.ir = multi_sensor_read_object
        # Allow scalar or per-sensor lists for black/white calibration.
        self.black = black
        self.white = white
        self.bias = bias

        if weights is not None:
            self.weights = list(weights)
        else:
            n = sensor_count
            if n is None:
                # Infer sensor count once. This keeps the driver "plug and play" when the array size changes.
                try:
                    n = len(self.ir.read())
                except Exception:
                    # Fall back to a safe default (7) if read is not available yet.
                    n = 7
            self.weights = self._make_symmetric_weights(n)

    @staticmethod
    def _make_symmetric_weights(n):
        """
        Create symmetric weights centered around 0.

        For odd n: integers from -k..+k.
        For even n: half-integers centered around 0 (e.g., n=8 -> [-3.5..+3.5]).
        """
        if n <= 0:
            return []
        center = (n - 1) / 2.0
        return [i - center for i in range(n)]

    def _ensure_weight_length(self, n):
        """Ensure self.weights matches number of sensor readings."""
        if len(self.weights) != n:
            self.weights = self._make_symmetric_weights(n)

    def calculate_error(self):
        """
        Calculate steering error using weighted centroid of normalized sensor readings.

        Normalization:
          norm = (reading - white) / (black - white), clamped to [0, 1]

        :return: Steering error (centered around 0, includes bias)
        :rtype: float
        """
        readings = self.ir.read()
        n = len(readings)
        if n == 0:
            return 0.0

        self._ensure_weight_length(n)

        numerator = 0.0
        denominator = 0.0

        # Decide if black/white are scalars or per-sensor sequences.
        # Use isinstance(list/tuple) instead of __len__ so this also works
        # under MicroPython where __len__ may not be exposed as an attribute.
        is_seq = isinstance(self.black, (list, tuple)) and isinstance(
            self.white, (list, tuple)
        )

        for idx, (w, reading) in enumerate(zip(self.weights, readings)):
            if is_seq:
                try:
                    b_i = float(self.black[idx])
                    w_i = float(self.white[idx])
                except (IndexError, TypeError, ValueError):
                    # Fallback: skip this sensor if calibration is malformed
                    continue
            else:
                b_i = float(self.black)
                w_i = float(self.white)

            denom_scale = (b_i - w_i)
            if denom_scale == 0:
                # Skip this sensor if calibration values are identical
                continue

            norm_value = (reading - w_i) / denom_scale
            if norm_value < 0.0:
                norm_value = 0.0
            elif norm_value > 1.0:
                norm_value = 1.0
            numerator += w * norm_value
            denominator += norm_value

        # If we don't see the line (all sensors near white), return 0 (straight).
        # Threshold of 0.15 prevents all-white noise from producing a spurious centroid.
        if denominator < 0.15:
            return 0.0

        error = numerator / denominator
        return error + self.bias

    def set_bias(self, bias):
        """Update steering bias dynamically."""
        self.bias = bias

    def set_weights(self, weights):
        """
        Override weights dynamically (must match sensor count).
        Useful if your sensor spacing is non-uniform or you removed/ignored edge sensors.
        """
        self.weights = list(weights)

    def get_raw_readings(self):
        """Get raw sensor readings for debugging or calibration."""
        return self.ir.read()

    @staticmethod
    def calibrate(multi_sensor_read_object):
        """
        Calculate average sensor reading for calibration.

        :return: Average ADC value across all sensors returned by read()
        :rtype: float
        """
        values = multi_sensor_read_object.read()
        if not values:
            return 0.0
        return sum(values) / len(values)
