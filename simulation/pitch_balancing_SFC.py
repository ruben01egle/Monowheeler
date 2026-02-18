import numpy as np
import matplotlib.pyplot as plt

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

# simulation parameter
time = 15.0
delta_t = 0.0001
steps = int(time / delta_t)

# physical parameters
MASS_PLATTFORM = 10.971  # kg
MASS = 13.971  # kg
g = 9.81  # m/s^2
MOI_YY_TAU = 0.6826  # kg*m^2
h_0 = 0.1123  # m
F_GP = g*MASS_PLATTFORM
F_G = g*MASS

# Servo PT1
SERVO_K = 0.85
SERVO_T = 0.045

# Arrays for simulation
t = np.zeros(steps)

theta = np.zeros(steps)
dot_theta = np.zeros(steps)
ddot_theta = np.zeros(steps)

p = np.zeros(steps)
dot_p = np.zeros(steps)
ddot_p = np.zeros(steps)
p_soll = np.zeros(steps)
p_max = 0.032
dot_p_max = 0.2

w = np.zeros(steps)

M_Stoerung = np.zeros(steps)
M_Verschiebung = np.zeros(steps)
M_Umfallen = np.zeros(steps)

# discrete control system variables\
ctrl_fa = 250
ctrl_Ta = 1/ctrl_fa

# Regler Daten
ctrl_t = 0.0
ctrl_theta = 0
ctrl_dot_theta = 0
ctrl_theta_sum = 0
ctrl_p = 0
ctrl_w = 0
ctrl_p_target = 0

THETA_UPPER = np.deg2rad(2.0) # Beispielwerte
THETA_LOWER = np.deg2rad(1.0)
DOT_THETA_UPPER = np.deg2rad(5.0)
DOT_THETA_LOWER = np.deg2rad(2.0)
T_SLOW_THRESHOLD = 0.5        # Zeit bis zum Wechsel in Slow
T_TRANSITION_MAX = 0.2        # Dauer der Blende

# Zustands-Enum (simuliert)
class Mode:
    SLOW = 0
    FAST = 1
    T_S2F = 2 # Transition Slow to Fast
    T_F2S = 3 # Transition Fast to Slow

# Controller Variablen
ctrl_mode = Mode.SLOW
ctrl_t_transition = 0.0
ctrl_t_slow = 0.0
ctrl_u_cmd = 0.0

# Systemmatrizen A, B
A = np.array([
    [0,                  1, 0],
    [F_G*h_0/MOI_YY_TAU, 0, 0],
    [-1,                 0, 0]
])

B = np.array([
    [0],
    [F_GP/MOI_YY_TAU],
    [0]
])

A_EXT = np.block([
    [A,        B],               # Obere linke (3x3) und obere rechte (3x1)
    [np.zeros((1,3)), np.array([[-1/SERVO_T]])]  # Untere linke (1x3) und untere rechte (1x1)
])
B_EXT = np.vstack([
    np.zeros((3,1)),
    np.array([[SERVO_K/SERVO_T]])
])


Kd_fast = np.array([[0.66638156,  0.12917211, -0.28520978,  1.02820138]])
Kd_slow = np.array([[0.66638156,  0.12917211, -0.28520978,  1.02820138]])

# Geschlossene Systemmatrix
Acl_ext_fast = A_EXT - B_EXT @ Kd_fast
Acl_ext_slow = A_EXT - B_EXT @ Kd_slow

# Pole = Eigenwerte
poles_fast, _ = np.linalg.eig(Acl_ext_fast)
poles_slow, _ = np.linalg.eig(Acl_ext_slow)

print("Poles of close loop system (fast SFC):")
print(poles_fast)
print("Poles of close loop system (slow SFC):")
print(poles_slow)

# Initial condition
theta[0] = 0.06

for n in range(1, steps):
    # -------------------------------------------
    # -------------------------------------------
    # discrete control
    if t[n - 1] - ctrl_t >= ctrl_Ta or t[n - 1] == 0:
        # Sensor fetch (mit deiner Totzeit-Simulation n-400)
        ctrl_p = p[n-1]
        ctrl_theta = theta[n-400] if n > 400 else theta[0]
        ctrl_dot_theta = dot_theta[n-400] if n > 400 else dot_theta[0]
        
        # Fehler-Integral (entspricht mThetaSum += -pTheta * mTa)
        ctrl_theta_sum += (ctrl_w - ctrl_theta) * ctrl_Ta

        # --- State Machine Logik ---
        if ctrl_mode == Mode.SLOW:
            if abs(ctrl_theta) > THETA_UPPER or abs(ctrl_dot_theta) > DOT_THETA_UPPER:
                ctrl_mode = Mode.T_S2F
                ctrl_t_transition = 0.0
        
        elif ctrl_mode == Mode.FAST:
            if abs(ctrl_theta) < THETA_LOWER and abs(ctrl_dot_theta) < DOT_THETA_LOWER:
                ctrl_t_slow += ctrl_Ta
                if ctrl_t_slow > T_SLOW_THRESHOLD:
                    ctrl_mode = Mode.T_F2S
                    ctrl_t_transition = 0.0
                    ctrl_t_slow = 0.0
            else:
                ctrl_t_slow = 0.0

        elif ctrl_mode == Mode.T_S2F:
            if ctrl_t_transition > T_TRANSITION_MAX:
                ctrl_mode = Mode.FAST
        
        elif ctrl_mode == Mode.T_F2S:
            if ctrl_t_transition > T_TRANSITION_MAX:
                ctrl_mode = Mode.SLOW

        # --- Berechnung der Stellgrößen ---
        # Hilfsfunktion für LQR Multiplikation: u = -(K * x)
        def calc_lqr(K):
            val = -(K[0,0]*ctrl_theta + K[0,1]*ctrl_dot_theta + 
                    K[0,2]*ctrl_theta_sum + K[0,3]*ctrl_p)
            return np.clip(val, -p_max, p_max)

        u_fast = calc_lqr(Kd_fast)
        u_slow = calc_lqr(Kd_slow)

        if ctrl_mode == Mode.FAST:
            ctrl_p_target = u_fast
        elif ctrl_mode == Mode.SLOW:
            ctrl_p_target = u_slow
        elif ctrl_mode == Mode.T_S2F:
            blend = np.clip(ctrl_t_transition / T_TRANSITION_MAX, 0.0, 1.0)
            ctrl_p_target = blend * u_fast + (1 - blend) * u_slow
            ctrl_t_transition += ctrl_Ta
        elif ctrl_mode == Mode.T_F2S:
            blend = np.clip(ctrl_t_transition / T_TRANSITION_MAX, 0.0, 1.0)
            ctrl_p_target = blend * u_slow + (1 - blend) * u_fast
            ctrl_t_transition += ctrl_Ta

        ctrl_t += ctrl_Ta
    # -------------------------------------------
    p_soll[n] = ctrl_p_target
    w[n] = ctrl_w

    # Servo-Verhalten
    dot_p[n-1] = np.clip((SERVO_K*p_soll[n] - p[n - 1])/SERVO_T, -dot_p_max, dot_p_max)
    p[n] = p[n-1] + dot_p[n-1]*delta_t

    if abs(p[n]) > p_max:
        p[n] = np.sign(p[n]) * p_max

    # Störung
    if t[n - 1] > 4.0 and t[n-1] < 4.1:
        M_Stoerung[n - 1] = 1
    elif t[n - 1] > 8.0:
        M_Stoerung[n - 1] = 0.2

    # Bewegung berechnen
    M_Verschiebung[n - 1] = MASS_PLATTFORM * g * p[n]
    M_Umfallen[n - 1] = MASS * g * h_0 * theta[n - 1]

    ddot_theta[n - 1] = (M_Verschiebung[n - 1] + M_Stoerung[n - 1] + M_Umfallen[n - 1]) / MOI_YY_TAU
    dot_theta[n] = dot_theta[n - 1] + ddot_theta[n - 1] * delta_t
    theta[n] = theta[n - 1] + dot_theta[n - 1] * delta_t

    if abs(theta[n]) > 0.175:
        theta[n] = np.sign(theta[n]) * 0.175

    t[n] = t[n - 1] + delta_t

# plotting
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Simulation Pitch", fontsize=16)

# oberer linker Plot: theta und w
axs[0, 0].plot(t, np.rad2deg(theta), label='Theta', color='red')
axs[0, 0].plot(t, np.rad2deg(w), label='ThetaTarget', linestyle='--',
                color='lightcoral')
axs[0, 0].set_title("Winkel Nicken")
axs[0, 0].set_xlabel("Zeit (s)")
axs[0, 0].set_ylabel("Winkel (°)")
axs[0, 0].legend()
axs[0, 0].grid()

# oberer rechter Plot: p und p_soll
axs[0, 1].plot(t, p_soll, label='TargetPosition',
                linestyle='--', color='lightcoral')
axs[0, 1].plot(t, p, label='Position', color='red')
axs[0, 1].set_title("Wheel-Dynamixel Radverschiebung")
axs[0, 1].set_xlabel("Zeit (s)")
axs[0, 1].set_ylabel("m")
axs[0, 1].legend()
axs[0, 1].grid()

# unterer linker Plot: Winkelgeschwindigkeit dot_theta
axs[1, 0].plot(t, np.rad2deg(dot_theta), label='DotTheta', color='red')
axs[1, 0].set_title("Winkelgeschwindigkeit Nicken")
axs[1, 0].set_xlabel('t in s')
axs[1, 0].set_ylabel('°/s')
axs[1, 0].legend()
axs[1, 0].grid()

# unterer rechter Plot leer lassen oder für spätere Nutzung
axs[1, 1].axis('off')

plt.tight_layout()
plt.show()