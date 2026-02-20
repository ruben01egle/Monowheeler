import numpy as np
import control
from framework.monowheeler_physics import MonowheelerConfig

cfg = MonowheelerConfig()
T_a = 1/250

A = np.array([
    [0,                                 1, 0, 0, 0],
    [cfg.F_G * cfg.H_0 / cfg.MOI_XX_TAU, 0, 0, 0, cfg.MOI_GYRO * cfg.W_K / cfg.MOI_XX_TAU],
    [0,                                 0, 0, 1, 0],
    [0,                                 0, 0, 0, 1],
    [0,                                 0, 0, 0, -1 / cfg.SERVO_ROLL_T]
])

# B-Matrix (5x1)
B = np.array([
    [0],
    [0],
    [0],
    [0],
    [cfg.SERVO_ROLL_K / cfg.SERVO_ROLL_T]
])

# Gewichtungsmatrizen Q, R
Q = np.diag([1/0.02, 1/0.02, 1/0.17, 100, 1/5])
R = np.array([[1/5*50000]])

# === Diskretisierung mit Zero-Order-Hold (ZOH) ===
sys_c = control.ss(A, B, np.eye(5), np.zeros((5,1)))
sys_d = control.c2d(sys_c, T_a, method='zoh')
Ad = sys_d.A
Bd = sys_d.B

# === LQR für diskretes System ===
Kd_LQR, _, _ = control.dlqr(Ad, Bd, Q, R)

Kd_real = np.array([[25, 4, -1.8, -0.15, 0.3]])

print("LQR Gain K:\n", Kd_LQR)
print("Real Gain K:\n", Kd_real)

# Geschlossene Systemmatrix
Acl_LQR = Ad - Bd @ Kd_LQR
poles_LQR, _ = np.linalg.eig(Acl_LQR)

Acl_real = Ad - Bd @ Kd_real
poles_real, _ = np.linalg.eig(Acl_real)

print("LQR closed-loop system poles:")
print(poles_LQR)

print("Real closed-loop system poles:")
print(poles_real)