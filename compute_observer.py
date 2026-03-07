# compute_observer.py
# Run this on a PC (requires numpy + scipy) to compute the discretized
# observer matrices AD and BD.  Paste the printed output into
# task_state_estimator.py where indicated.
#
# State:   x = [s (m), psi (rad), OmL (rad/s), OmR (rad/s)]
# Input:   u = [uL, uR]  (volts)
# Output:  y = [sL (m), sR (m), psi (rad), psi_dot (rad/s)]
# Ext. input: u* = [uL_V, uR_V, sL, sR, psi, psi_dot]  (len 6)
#
# Discrete observer:
#   x_hat_{k+1} = AD @ x_hat_k + BD @ u*_k

import numpy as np
from scipy.linalg import expm, solve_continuous_are

# ------------------------------------------------------------------ #
# Romi parameters (from Lab 0x06)
# ------------------------------------------------------------------ #
r   = 0.035   # m   wheel radius
w   = 0.140   # m   wheelbase
tau = 0.05    # s   motor time constant
Km  = 3.49    # rad/(V·s)  motor gain

Ts  = 0.02    # s   sample time (20 ms task period)

# ------------------------------------------------------------------ #
# Continuous-time system matrices  ẋ = A x + B u
# State: [s, psi, OmL, OmR]
# ------------------------------------------------------------------ #
A = np.array([
    [0, 0,  r/2,    r/2   ],   # ds/dt    = r/2*(OmL + OmR)
    [0, 0, -r/w,    r/w   ],   # dpsi/dt  = r/w*(OmR - OmL)
    [0, 0, -1/tau,  0     ],   # dOmL/dt  = Km/tau*uL - 1/tau*OmL
    [0, 0,  0,     -1/tau ],   # dOmR/dt  = Km/tau*uR - 1/tau*OmR
])

B = np.array([
    [0,       0      ],
    [0,       0      ],
    [Km/tau,  0      ],
    [0,       Km/tau ],
])

# ------------------------------------------------------------------ #
# Output matrix  y = C x + D u
# Output: [sL, sR, psi, psi_dot]
# ------------------------------------------------------------------ #
C = np.array([
    [1, -w/2, 0,    0    ],   # sL = s - (w/2)*psi
    [1,  w/2, 0,    0    ],   # sR = s + (w/2)*psi
    [0,  1,   0,    0    ],   # psi
    [0,  0,  -r/w,  r/w  ],   # psi_dot = r/w*(OmR - OmL)
])

D = np.zeros((4, 2))

# ------------------------------------------------------------------ #
# Controllability and observability check
# ------------------------------------------------------------------ #
Co = np.hstack([B, A@B, A@A@B, A@A@A@B])
Ob = np.vstack([C, C@A, C@A@A, C@A@A@A])
print("Controllability rank:", np.linalg.matrix_rank(Co), "(should be 4)")
print("Observability rank:  ", np.linalg.matrix_rank(Ob), "(should be 4)")

# ------------------------------------------------------------------ #
# Observer gain via Continuous Algebraic Riccati Equation (CARE)
# This is equivalent to an optimal Kalman observer and is numerically
# robust where pole placement (place_poles) fails on degenerate A.
#
# Tune Q_w (process noise) and R_v (measurement noise) to shape
# how aggressively the observer trusts measurements vs the model.
#   - Increase Q_w[i,i]  → observer tracks state i more from measurements
#   - Increase R_v[j,j]  → observer trusts output j less
# ------------------------------------------------------------------ #
Q_w = np.diag([1e-2, 1e-2, 1.0, 1.0])   # process noise: [s, psi, OmL, OmR]
R_v = np.diag([1e-4, 1e-4, 1e-4, 1e-4]) # measurement noise: [sL, sR, psi, psi_dot]

# Solve: A*P + P*A^T - P*C^T*R_v^-1*C*P + Q_w = 0
P = solve_continuous_are(A, C.T, Q_w, R_v)
L = P @ C.T @ np.linalg.inv(R_v)        # shape (4, 4)

print("\nObserver gain L:\n", np.round(L, 6))
print("Closed-loop observer poles:", np.round(np.sort(np.linalg.eigvals(A - L @ C).real), 4))

# ------------------------------------------------------------------ #
# Observer closed-loop matrices
# ------------------------------------------------------------------ #
Ao = A - L @ C                       # (4x4)
Bo = np.hstack([B - L @ D, L])       # (4x6) = [B, L]  since D=0

# ------------------------------------------------------------------ #
# Discretize:  AD = e^(Ao*Ts),  BD = Ao^-1 (AD - I) Bo
# ------------------------------------------------------------------ #
AD = expm(Ao * Ts)
BD = np.linalg.solve(Ao, (AD - np.eye(4)) @ Bo)

print("\nAD (4x4) =")
print(np.round(AD, 8).tolist())

print("\nBD (4x6) =")
print(np.round(BD, 8).tolist())

print("\n--- Copy the AD and BD lists into task_state_estimator.py ---")
