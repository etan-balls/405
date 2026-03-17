# task_state_estimator.py
# Discretized Luenberger state observer for Romi.
#
# State:    x = [s (m), psi (rad), OmL (rad/s), OmR (rad/s)]
# Ext input: u* = [uL_V, uR_V, sL_meas, sR_meas, psi_meas, psidot_meas]  (len 6)
# Note: motor inputs are in VOLTS. Convert %PWM using _K_PWM before passing.
#
# Observer update (runs every Ts = 20 ms):
#   x_hat_{k+1} = AD @ x_hat_k + BD @ u*_k
#
# ------------------------------------------------------------------ #
# HOW TO GET AD AND BD:
#   1. Run compute_observer.py on your PC (requires numpy + scipy).
#   2. Copy-paste the printed AD and BD lists below.
# ------------------------------------------------------------------ #

import micropython

try:
    from ulab import numpy as np
    _HAVE_ULAB = True
except ImportError:
    _HAVE_ULAB = False

import math

# ------------------------------------------------------------------ #
# Physical constants (must match compute_observer.py)
# ------------------------------------------------------------------ #
_M_PER_COUNT  = 0.00015303   # metres per encoder count  (2*pi*35mm / 1437.07 / 1000)
_RAD_PER_DEG  = math.pi / 180.0
_K_PWM        = 0.974 / 30.0  # V per %PWM  (tune if needed)

# ------------------------------------------------------------------ #
# Discretized observer matrices — paste output of compute_observer.py
# AD and BD are defined inside __init__ as locals so they are freed
# after being converted to ulab arrays, saving ~500 bytes of heap.

# ------------------------------------------------------------------ #
# Task states
# ------------------------------------------------------------------ #
S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)


class task_state_estimator:
    """
    Cooperative task that runs the discretized Luenberger observer.

    Parameters
    ----------
    left_enc  : encoder object  (has get_position(), get_velocity())
    right_enc : encoder object
    imu_heading_share   : Share('f')  heading in DEGREES from BNO055
    imu_yawrate_share   : Share('f')  yaw-rate in DEG/S from BNO055
    effort_L_share      : Share('f')  current left  motor effort (%PWM)
    effort_R_share      : Share('f')  current right motor effort (%PWM)
    psi_hat_share       : Share('f')  OUT – estimated heading (rad)
    s_hat_share         : Share('f')  OUT – estimated arc-length (m)
    omL_hat_share       : Share('f')  OUT – estimated left  wheel speed (rad/s)
    omR_hat_share       : Share('f')  OUT – estimated right wheel speed (rad/s)
    """

    def __init__(self,
                 left_enc, right_enc,
                 imu_heading_share, imu_yawrate_share,
                 effort_L_share, effort_R_share,
                 psi_hat_share, s_hat_share,
                 omL_hat_share, omR_hat_share):

        self._left  = left_enc
        self._right = right_enc

        self._imu_h  = imu_heading_share
        self._imu_gz = imu_yawrate_share
        self._uL     = effort_L_share
        self._uR     = effort_R_share

        self._psi_out = psi_hat_share
        self._s_out   = s_hat_share
        self._omL_out = omL_hat_share
        self._omR_out = omR_hat_share

        # Build ulab matrices once. Data defined as locals so the lists
        # are freed after conversion (module-level lists stay in heap forever).
        _ad = [
            [ 0.75363828,  0.0,         0.00024891,  0.00024891],
            [ 0.0,         0.81791656, -0.00296983,  0.00296983],
            [-0.0001031,   0.0006063,   0.55802321,  0.1122968 ],
            [-0.0001031,  -0.0006063,   0.1122968,   0.55802321],
        ]
        _bd = [
            [ 0.00019526,  0.00019526,  0.12318086,  0.12318086,  0.0,         0.0       ],
            [-0.00245028,  0.00245028, -0.01271053,  0.01271053,  0.18029125,  0.00332566],
            [ 1.05407798,  0.09650504,  9.387e-05,   9.23e-06,   -0.00060033, -0.56378613],
            [ 0.09650504,  1.05407798,  9.23e-06,    9.387e-05,   0.00060033,  0.56378613],
        ]
        if _HAVE_ULAB:
            self._AD = np.array(_ad)
            self._BD = np.array(_bd)
            self._xh = np.array([0.0, 0.0, 0.0, 0.0])  # 1D vector (ulab has no @ operator)
        else:
            # Fallback: pure-Python lists (slow; replace with real ulab ASAP)
            self._AD = _ad
            self._BD = _bd
            self._xh = [0.0, 0.0, 0.0, 0.0]

        # Track cumulative distance for sL / sR measurements
        self._sL = 0.0
        self._sR = 0.0
        self._psi0 = None   # heading reference (first valid IMU reading)

        self._state = S0_INIT
        print("State Estimator Task instantiated")

    # -------------------------------------------------------------- #
    # Internal helpers
    # -------------------------------------------------------------- #

    def _mat_vec_add(self, A, B, x, u):
        """Compute A @ x + B @ u for plain-Python list fallback (4-state, 6-input)."""
        result = [0.0] * 4
        for i in range(4):
            s = 0.0
            for j in range(4):
                s += A[i][j] * x[j]
            for j in range(6):
                s += B[i][j] * u[j]
            result[i] = s
        return result

    def _update_ulab(self, u_star):
        """One observer step using ulab (fast path). Uses np.dot() — ulab has no @ operator."""
        u_vec = np.array(u_star)
        self._xh = np.dot(self._AD, self._xh) + np.dot(self._BD, u_vec)

    def _update_python(self, u_star):
        """One observer step using pure-Python lists (slow fallback)."""
        self._xh = self._mat_vec_add(self._AD, self._BD, self._xh, u_star)

    # -------------------------------------------------------------- #
    # Task generator
    # -------------------------------------------------------------- #

    def run(self):
        while True:
            if self._state == S0_INIT:
                # Zero encoders so cumulative distances start at 0
                self._left.zero()
                self._right.zero()
                self._sL = 0.0
                self._sR = 0.0
                self._psi0 = None
                self._state = S1_RUN

            elif self._state == S1_RUN:
                # -- 1. Update encoders (they must also be updated by whoever
                #       calls update() in the main loop; this read is safe
                #       because we only call get_* here).
                # Both encoders count negative going forward; negate so the
                # observer receives sL, sR positive going forward (matching the
                # model: sL = s - (w/2)*psi, sR = s + (w/2)*psi, both positive).
                sL_m = -self._left.get_position()  * _M_PER_COUNT
                sR_m = -self._right.get_position() * _M_PER_COUNT

                # -- 2. IMU measurements (degrees → radians)
                h_deg  = self._imu_h.get()
                gz_dps = self._imu_gz.get()

                if h_deg is None:
                    h_deg  = 0.0
                if gz_dps is None:
                    gz_dps = 0.0

                # Latch heading reference on first valid reading
                if self._psi0 is None:
                    self._psi0 = h_deg

                psi_meas     = (h_deg - self._psi0) * _RAD_PER_DEG
                psidot_meas  = gz_dps * _RAD_PER_DEG

                # -- 3. Motor efforts (%PWM) → convert to volts for observer
                uL = self._uL.get()
                uR = self._uR.get()
                if uL is None: uL = 0.0
                if uR is None: uR = 0.0
                uL_V = uL * _K_PWM
                uR_V = uR * _K_PWM

                # -- 4. Extended input vector
                u_star = [uL_V, uR_V, sL_m, sR_m, psi_meas, psidot_meas]

                # -- 5. Observer step
                if _HAVE_ULAB:
                    self._update_ulab(u_star)
                    s_h   = float(self._xh[0])
                    psi_h = float(self._xh[1])
                    omL_h = float(self._xh[2])
                    omR_h = float(self._xh[3])
                else:
                    self._update_python(u_star)
                    s_h, psi_h, omL_h, omR_h = self._xh

                # -- 6. Publish estimates
                self._psi_out.put(psi_h)
                self._s_out.put(s_h)
                self._omL_out.put(omL_h)
                self._omR_out.put(omR_h)

            yield self._state
