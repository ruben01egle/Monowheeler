import numpy as np
import matplotlib.pyplot as plt

# Step size and simulation time
time = 30.0
delta_t = 0.0001
steps = int(time / delta_t)

# physical parameters
MASS_PLATTFORM = 10.971  # kg
MASS = 13.971  # kg
g = 9.81  # m/s^2
h_0 = 0.1123  # m
F_G = MASS*g
MOI_XX_TAU = 0.2489
MOI_ZZ = 0.5017

n_k = 5000
w_k = n_k/60*2*np.pi
MOI_GYRO = 3.3991489e-03

# Servo PT1
SERVO_K = 0.85
SERVO_T = 0.1

v = 0.5                        #konst Geschwindigkeit in Fahrtrichtung

friction_roll_coeff = 100
friction_yaw_coeff = 0.005
static_friction_yaw_threshhold_M = 0.5
static_friction_yaw_threshhold_dot_phi = 0.04

# Arrays for simulation
t = np.zeros(steps)

ddot_psi_K = np.zeros(steps)
dot_psi_K = np.zeros(steps)
psi_K = np.zeros(steps)

ddot_phi = np.zeros(steps)
dot_phi = np.zeros(steps)
phi = np.zeros(steps)

ddot_psi = np.zeros(steps)
dot_psi = np.zeros(steps)
psi = np.zeros(steps)

M_kphi = np.zeros(steps)
M_kpsi = np.zeros(steps)

M_psi_ges = np.zeros(steps)

dot_psi_inert = np.zeros(steps)
psi_inert = np.zeros(steps)

n1 = np.zeros(steps)
n2 = np.zeros(steps)


dot_psi_K_soll = np.zeros(steps)
dot_psi_K_max = 5                          # max Winkelgeschwindigkeit Servo -> 5 rad/s
psi_K_max = 0.25

phi_target = np.zeros(steps)                       # Sollwert Phi

M_Stoerung_Phi = np.zeros(steps)             # Störung
M_Stoerung_Psi = np.zeros(steps) 

# discrete control system variables\
ctrl_fa = 250
ctrl_Ta = 1/ctrl_fa

A = np.array([
    [0                 , 1, 0, 0, 0                      ],
    [F_G*h_0/MOI_XX_TAU, 0, 0, 0, MOI_GYRO*w_k/MOI_XX_TAU],
    [0                 , 0, 0, 0, 1                      ],
    [0                 , 0, 1, 0, 0                      ],
    [0                 , 0, 0, 0, -1/SERVO_T             ]
])

B = np.array([
    [0],
    [0],
    [0],
    [0],
    [SERVO_K/SERVO_T]
])

Kd = np.array([[25, 4, -1.8, -0.15, 0.3]])
# Geschlossene Systemmatrix
Acl = A - B @ Kd

# Pole = Eigenwerte
poles, _ = np.linalg.eig(Acl)

print("Poles of closed loop system:")
print(poles)

# Regler Daten
ctrl_t = 0.0
ctrl_phi = 0
ctrl_dot_phi = 0
ctrl_phi_target = 0
ctrl_w = 0
ctrl_dot_psi_K_target = 0
ctrl_dot_psi_K = 0
ctrl_psi_K = 0
ctrl_psi_K_sum = 0
ctrl_GF = 0.98

# Initial condition
phi[0] = np.deg2rad(0.5)
ctrl_phi_target = phi[0]
ctrl_w = 0

for n in range(1, steps):
    # -------------------------------------------
    # discrete control
    if t[n - 1] - ctrl_t >= ctrl_Ta:
        
        # Sensor fetch
        ctrl_phi = phi[n-1]
        ctrl_dot_phi = dot_phi[n-1]
        ctrl_dot_psi_K = dot_psi_K[n-1]
        ctrl_psi_K = psi_K[n-1]
        ctrl_psi_K_sum += ctrl_psi_K*ctrl_Ta

        ctrl_phi_target = (1-ctrl_GF)*ctrl_w + ctrl_GF*ctrl_phi_target
    
        ctrl_dot_psi_K_target = -np.clip(Kd[0, 0]*(ctrl_phi-ctrl_phi_target)
                                       + Kd[0, 1]*ctrl_dot_phi
                                       + Kd[0, 2]*ctrl_psi_K 
                                       + Kd[0, 3]*ctrl_psi_K_sum
                                       + Kd[0, 4]*ctrl_dot_psi_K, 
                                       -dot_psi_K_max, dot_psi_K_max)

        ctrl_t += ctrl_Ta

    dot_psi_K_soll[n] = ctrl_dot_psi_K_target
    phi_target[n] = ctrl_phi_target
    
    # Servo-Verhalten
    ddot_psi_K[n-1] = np.clip((SERVO_K*dot_psi_K_soll[n] - dot_psi_K[n - 1])/SERVO_T, -dot_psi_K_max/delta_t, dot_psi_K_max/delta_t)
    dot_psi_K[n] = dot_psi_K[n-1] + ddot_psi_K[n-1]*delta_t
    psi_K[n] = psi_K[n-1] + dot_psi_K[n-1]*delta_t

    if abs(psi_K[n-1]) >= psi_K_max:
       dot_psi_K[n] = 0
    
    # Störung
    if t[n-1] > 15 and t[n-1] < 15.5:          
        M_Stoerung_Phi[n] = -0.05
    else:
        M_Stoerung_Phi[n] = +0.05
    
    # Bewegungssituation
    # Kreiselmomente
    M_kphi[n-1] = MOI_GYRO*w_k*(dot_psi[n-1] + dot_psi_K[n])
    M_kpsi[n-1] = -MOI_GYRO*w_k*dot_phi[n-1]

    # BGL
    # dpt_phi mit Gleitreibung
    ddot_phi[n-1] = 1/MOI_XX_TAU * (M_kphi[n-1] + MASS*v*dot_psi[n-1]*h_0 + F_G*np.sin(phi[n-1])*h_0 + M_Stoerung_Phi[n]) - np.sign(dot_phi[n-1])*friction_roll_coeff*dot_phi[n-1]**2
    # Integration phi
    dot_phi[n] = dot_phi[n-1] + ddot_phi[n-1]*delta_t
    phi[n] = phi[n-1] + dot_phi[n-1]*delta_t
    
    if abs(phi[n]) >= 0.25:                         # Kippwinkel begrenzen
        phi[n] = np.sign(phi[n])*0.25

    # dot_psi Gleitreibung und Haftreibung und Integration psi
    M_psi_ges[n-1] = M_kpsi[n-1] + M_Stoerung_Psi[n] - np.sign(dot_psi[n-1])*0.005*dot_psi[n-1]**2
    if abs(M_psi_ges[n-1]) < static_friction_yaw_threshhold_M  and abs(dot_psi[n-1]) < static_friction_yaw_threshhold_dot_phi:
        ddot_psi[n-1] = 0
        dot_psi[n] = 0
    else:
        ddot_psi[n-1] = 1/MOI_ZZ*M_psi_ges[n-1]
        dot_psi[n] = dot_psi[n-1] + ddot_psi[n-1]*delta_t
 
    psi[n] = psi[n-1] + dot_psi[n-1]*delta_t
    
    # Integration psi_K
    psi_K[n] = psi_K[n-1] + dot_psi_K[n-1]*delta_t 

    # Kreisbahn im Inertialsystem als Projektion auf xy Ebene darstellen
    dot_psi_inert[n] = dot_psi[n-1]*np.cos(phi[n-1]) 
    psi_inert[n] = psi_inert[n-1] + dot_psi_inert[n-1]*delta_t

    n1[n] = n1[n-1] + v*np.cos(psi_inert[n-1])*delta_t
    n2[n] = n2[n-1] + v*np.sin(psi_inert[n-1])*delta_t

    #Zeit
    t[n] = t[n-1] + delta_t


rad2deg = 180 / np.pi
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Simulation Roll", fontsize=16)

# --- Plot 1: Rollwinkel ---
ax = axs[0, 0]
ax.plot(t, phi * rad2deg, label='Phi', color='blue')
ax.plot(t, phi_target * rad2deg, label='PhiTarget', linestyle='--', color='dodgerblue')
ax.set_title("Winkel Rollen")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("Winkel (°)")
ax.grid(True)
ax.legend()

# --- Plot 2: Gyro-Dynamixel (psi_K) ---
ax = axs[0, 1]
ax.plot(t, psi_K * rad2deg, label='Position', color='green', zorder=3)
ax.plot(t, dot_psi_K * rad2deg, label='Velocity', color='blue', zorder=2)
ax.plot(t, dot_psi_K_soll * rad2deg, label='TargetVelocity', linestyle='--', color='dodgerblue', zorder=1)
ax.set_title("Gyro-Dynamixel")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("Winkel (°) / °/s")
ax.grid(True)
ax.legend()

# --- Plot 3: Winkelgeschwindigkeit des Rollens ---
ax = axs[1, 0]
ax.plot(t, dot_phi * rad2deg, label='DotPhi', color='blue')
ax.set_title("Winkelgeschwindigkeit Rollen")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("°/s")
ax.grid(True)
ax.legend()

# --- Plot 4: Gierwinkel ---
ax = axs[1, 1]
ax.plot(t, dot_psi * rad2deg, label='DotPsi', color='purple')
ax.set_title("Winkelgeschwindigkeit Gieren")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("°/s")
ax.grid(True)
ax.legend()

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()