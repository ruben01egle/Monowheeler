import numpy as np
import control

from framework.monowheeler_physics import MonowheelerConfig

'''
J_yt*ddot_theta = F_GP*p + F_G*h0*sin(theta)
Linearisierung im AP theta = 0:
J_yt*ddot_theta = F_GP*p + F_G*h0*theta
Zustandsvariablen: x1 = theta, x2 = dot_theta, u = p
dot_x1 = x2
dot_x2 = 1/J_yt*(F_GP*u + F_G*h0*x1)

dot_x           A                   x        B           u
[dot_x1]    =   [0,           1]  * [x1]   + [0        ]*u
[dot_x2]        [F_G*h0/J_yt, 0]    [x2]     [F_GP/J_yt]

Erweiterung um Integrierten Fehler: x3 = sum(theta_ref - theta)
dot_x3 = theta_ref - theta
dot_x           A                      x        B           u
[dot_x1]        [0,           1, 0]    [x1]     [0        ]     [0]
[dot_x2]    =   [F_G*h0/J_yt, 0, 0]  * [x2]  +  [F_GP/J_yt]*u + [0]*theta_ref
[dot_x3]        [-1,          0, 0]    [x3]     [0        ]     [1]


Erweiterung um Servo Verzogerung zu beachten:
x_4 = p, u = p_cmd
mit p als Radverschiebung und p als Eingang zum Aktor
Zusammenhang p und p_cmd als PT1:
K*p_cmd = T*dot_p + p
[dot_x1]        [0,           1, 0, 0   ]    [x1]     [0        ]
[dot_x2]    =   [F_G*h0/J_yt, 0, 0, 0   ]  * [x2]  +  [F_GP/J_yt]*u
[dot_x3]        [-1,          0, 0, 0   ]    [x3]     [0        ]
[dot_x4]        [-1,          0, 0, -1/T]    [x4]     [K/T      ]
'''

cfg = MonowheelerConfig()
T_a = 1/250

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


# Gewichtungsmatrizen Q, R
Q = np.diag([1/0.02, 1/0.02, 15000])
R = np.array([[1/0.002*340]])

Q = np.block([
    [Q,           np.zeros((3, 1))],
    [np.zeros((1, 3)), np.array([[1/0.002*500]])]
])

# === Diskretisierung mit Zero-Order-Hold (ZOH) ===
sys_c = control.ss(A, B, np.eye(4), np.zeros((4,1)))
sys_d = control.c2d(sys_c, T_a, method='zoh')
Ad = sys_d.A
Bd = sys_d.B

# === LQR für diskretes System ===
Kd_fast, _, _ = control.dlqr(Ad, Bd, Q, R)
Kd_slow = np.array([[0.45, 0.5, 1.0, 1.0]])*Kd_fast

print("Fast LQR Gain K:\n", Kd_fast)
Acl_fast = Ad - Bd @ Kd_fast
poles_fast, _ = np.linalg.eig(Acl_fast)

print("Slow LQR Gain K:\n", Kd_slow)
Acl_slow = Ad - Bd @ Kd_slow
poles_slow, _ = np.linalg.eig(Acl_slow)

print("Fast closed-loop system poles:")
print(poles_fast)
print("Slow closed-loop system poles:")
print(poles_slow)