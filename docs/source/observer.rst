State Observer
==============

The firmware includes a **discretized Luenberger observer** (``task_state_estimator.py``)
that fuses encoder odometry and IMU measurements to estimate the robot's full state at
50 Hz. The observer gain is designed off-line using ``compute_observer.py`` (requires
``numpy`` and ``scipy`` on a PC).

State & Output Definitions
---------------------------

.. list-table::
   :widths: 15 20 65
   :header-rows: 1

   * - Symbol
     - Units
     - Description
   * - *s*
     - m
     - Arc-length — total distance travelled along the robot's path
   * - *ψ*
     - rad
     - Heading — angle from the initial forward direction
   * - *Ω*\ :sub:`L`
     - rad/s
     - Left wheel angular speed
   * - *Ω*\ :sub:`R`
     - rad/s
     - Right wheel angular speed
   * - *u*\ :sub:`L`, *u*\ :sub:`R`
     - V
     - Left and right motor voltages (converted from %PWM via *K*\ :sub:`PWM`)

Physical Parameters
-------------------

.. list-table::
   :widths: 20 15 65
   :header-rows: 1

   * - Parameter
     - Value
     - Description
   * - *r*
     - 35 mm
     - Wheel radius
   * - *w*
     - 141 mm
     - Wheelbase (track width between contact patches)
   * - *τ*
     - 50 ms
     - Motor first-order time constant
   * - *K*\ :sub:`m`
     - 3.49 rad/(V·s)
     - Motor gain
   * - *T*\ :sub:`s`
     - 20 ms
     - Discrete sample period

Continuous-Time Plant Model
----------------------------

The Romi kinematics and motor dynamics combine into a 4th-order linear system:

.. raw:: html

   <div class="math-block">ẋ = A·x + B·u

A = ⎡ 0    0    r/2    r/2  ⎤     B = ⎡ 0       0     ⎤
    ⎢ 0    0   −r/w    r/w  ⎥         ⎢ 0       0     ⎥
    ⎢ 0    0  −1/τ     0    ⎥         ⎢ Km/τ    0     ⎥
    ⎣ 0    0    0     −1/τ  ⎦         ⎣ 0       Km/τ  ⎦</div>

The output vector **y** = [*s*\ :sub:`L`, *s*\ :sub:`R`, *ψ*, *ψ̇*] is what we can
measure from the encoders and IMU:

.. raw:: html

   <div class="math-block">C = ⎡ 1  −w/2   0    0   ⎤    sL = s − (w/2)·ψ
       ⎢ 1   w/2   0    0   ⎥    sR = s + (w/2)·ψ
       ⎢ 0    1    0    0   ⎥    ψ
       ⎣ 0    0   −r/w  r/w ⎦    ψ̇ = (r/w)·(ΩR − ΩL)</div>

Observer Gain Design (CARE)
----------------------------

The observer gain **L** is found by solving the **Continuous Algebraic Riccati
Equation** — equivalent to an optimal Kalman filter design:

.. raw:: html

   <div class="math-block">A·P + P·Aᵀ − P·Cᵀ·Rv⁻¹·C·P + Qw = 0

L = P·Cᵀ·Rv⁻¹     (shape 4×4)

Qw = diag([1e-2, 1e-2, 1.0, 1.0])   ← process noise  [s, ψ, ΩL, ΩR]
Rv = diag([1e-4, 1e-4, 1e-4, 1e-4]) ← measurement noise [sL, sR, ψ, ψ̇]</div>

**Tuning intuition:**

- Increase ``Qw[i,i]`` → observer trusts sensor *i* more, tracks it more aggressively.
- Increase ``Rv[j,j]`` → observer trusts output *j* less, relies more on the model.

Discretization
--------------

The closed-loop observer matrices are discretized at *T*\ :sub:`s` = 20 ms using the
matrix exponential:

.. raw:: html

   <div class="math-block">Ao = A − L·C             (closed-loop observer, 4×4)
Bo = [B − L·D,  L]       (extended input: [motor voltages, sensor corrections], 4×6)

AD = expm(Ao · Ts)
BD = Ao⁻¹ · (AD − I) · Bo</div>

These matrices are computed once on a PC by ``compute_observer.py`` and hard-coded
into ``task_state_estimator.py``.

Observer Update Equation
-------------------------

Every 20 ms, the estimator executes one step:

.. raw:: html

   <div class="math-block">u* = [uL_V, uR_V, sL_meas, sR_meas, ψ_meas, ψ̇_meas]   ← 6-element extended input

x̂ₖ₊₁ = AD · x̂ₖ + BD · u*ₖ</div>

The matrix multiply is done with **ulab** (MicroPython's numpy) for performance.
A pure-Python fallback is included for boards without ulab.

.. code-block:: python

   # Fast path (ulab):
   u_vec    = np.array(u_star)
   self._xh = np.dot(self._AD, self._xh) + np.dot(self._BD, u_vec)

   # Slow fallback (pure Python):
   self._xh = self._mat_vec_add(self._AD, self._BD, self._xh, u_star)

IMU Heading Unwrapping
-----------------------

The BNO055 wraps heading to [0°, 360°). Without correction, a 359° → 1° transition
would look like a −358° step to the observer. The estimator unwraps this by computing
the shortest-path delta:

.. code-block:: python

   d = ((h_deg - h_deg_prev + 180.0) % 360.0) - 180.0
   h_deg = h_deg_prev + d

Re-running the Observer Design
--------------------------------

To recompute AD and BD (e.g. if you change wheel geometry or motor parameters):

.. code-block:: bash

   python compute_observer.py

Copy the printed ``AD`` and ``BD`` lists into ``task_state_estimator.py`` where
the ``_ad`` and ``_bd`` locals are defined in ``__init__``.
