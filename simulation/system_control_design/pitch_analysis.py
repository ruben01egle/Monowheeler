import numpy as np
from numpy.linalg import matrix_rank, eig

from framework.monowheeler_physics import MonowheelerConfig

cfg = MonowheelerConfig()

A = np.array([
    [0,                                 1, 0, 0],
    [cfg.F_G * cfg.H_0 / cfg.MOI_YY_TAU, 0, 0, cfg.F_GP / cfg.MOI_YY_TAU],
    [-1,                                0, 0, 0],
    [0,                                 0, 0, -1 / cfg.SERVO_PITCH_T]
])

B = np.array([
    [0],
    [0],
    [0],
    [cfg.SERVO_PITCH_K / cfg.SERVO_PITCH_T]
])

# Eigenwerte
eigvals, eigvecs = eig(A)
print("Eigenvalues A:\n", eigvals)

# --- Steuerbarkeit ---
n = A.shape[0]
Ctrb = B
for i in range(1, n):
    Ctrb = np.hstack((Ctrb, np.linalg.matrix_power(A, i) @ B))

rank_ctrb = matrix_rank(Ctrb)
print("Controllability matrix shape:", Ctrb.shape)
print("Rank of the controllability matrix:", rank_ctrb, "out of", n)

if rank_ctrb == n:
    print("➡️ The system is fully controllable.")
else:
    print("⚠️ The system is NOT fully controllable.")
