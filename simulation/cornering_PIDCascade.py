import numpy as np
import matplotlib.pyplot as plt

# Step size and simulation time
time = 20.0
delta_t = 0.0001
steps = int(time / delta_t)

# physical parameters
MASS_PLATTFORM = 10.971  # kg
MASS = 13.971  # kg
g = 9.81  # m/s^2
h_0 = 0.1123  # m
F_G = MASS*g
MOI_XX_TAU = 0.2489
MOI_YY_TAU = 0.6826  # kg*m^2
MOI_ZZ = 0.5017

n_k = 5000
w_k = n_k/60*2*np.pi
MOI_GYRO = 3.3991489e-03

# Servo PT1
SERVO_K = 0.85
SERVO_T = 0.13

v = 1.5                        #konst Geschwindigkeit in Fahrtrichtung

friction_roll_coeff = 200
friction_yaw_coeff = 0.2
static_friction_yaw_threshhold_M = 0.08
static_friction_yaw_threshhold_dot_phi = 0.02

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
dot_psi_K_max = 5                          # max w Servo -> 5 rad/s
psi_K_max = 0.25

phi_target = np.zeros(steps)                       # Sollwert Phi
dot_psi_target = np.zeros(steps)

M_Stoerung_Phi = np.zeros(steps)             # Störung
M_Stoerung_Psi = np.zeros(steps) 

# PID controller parameters
KP_ROLL = 15
KD_ROLL = 3
KI_ROLL = 3.0

KP_TRAJ_BAL = 0.02
KD_TRAJ_BAL = 0.003
KI_TRAJ_BAL = 0.01

KP_TRAJ_YAW = 0.8
KD_TRAJ_YAW = 0.15
KI_TRAJ_YAW = 0

# discrete control system variables\
ctrl_fa = 250
ctrl_Ta = 1/ctrl_fa

# Regler Daten
ctrl_t = 0.0
ctrl_phi = 0
ctrl_dot_phi = 0
ctrl_phi_target = 0
ctrl_dot_phi_target = 0
ctrl_e_phi = 0
ctrl_dot_e_phi = 0
ctrl_sum_e_phi = 0
ctrl_e_psiK = 0
ctrl_dot_e_psiK = 0
ctrl_sum_e_psiK = 0
ctrl_e_dotpsi = 0
ctrl_dot_e_dotpsi = 0
ctrl_sum_e_dotpsi = 0
ctrl_dotpsi_target = 0
ctrl_dot_psi_K_target = 0
ctrl_dot_psi_K = 0
ctrl_psi_K = 0

corner = False

# Initial condition
phi[0] = np.deg2rad(1)
ctrl_phi_target = 0.00

for n in range(1, steps):
    # -------------------------------------------
    # discrete control
    if t[n - 1] - ctrl_t >= ctrl_Ta:
        
        # Sensor fetch
        ctrl_phi = phi[n-1]
        ctrl_dot_phi = dot_phi[n-1]
        ctrl_dot_psi = dot_psi[n-1]
        ctrl_dot_psi_K = dot_psi_K[n-1]
        ctrl_psi_K = psi_K[n-1]

        if ctrl_t > 5 and ctrl_t < 11:
            corner = True
            ctrl_dotpsi_target = -np.deg2rad(7)
            ctrl_sum_e_psiK = 0
        #elif ctrl_t > 16 and ctrl_t < 20:
        #    corner = True
        #    ctrl_dotpsi_target = np.deg2rad(5)
        #    ctrl_sum_e_psiK = 0
        else:
            corner = False
            ctrl_dotpsi_target = 0

        if corner:
            ctrl_e_dotpsi_new = ctrl_dotpsi_target - ctrl_dot_psi
            ctrl_dot_e_dotpsi = (ctrl_e_dotpsi_new - ctrl_e_dotpsi)/ctrl_Ta
            ctrl_e_dotpsi = ctrl_e_dotpsi_new
            if (np.abs(ctrl_phi_target) < np.deg2rad(5)):
                ctrl_sum_e_dotpsi += ctrl_e_dotpsi*ctrl_Ta
            ctrl_dot_phi_target = -np.clip(KD_TRAJ_YAW*ctrl_dot_e_dotpsi + KP_TRAJ_YAW*ctrl_e_dotpsi + KI_TRAJ_YAW*ctrl_sum_e_dotpsi, np.deg2rad(-5), np.deg2rad(5))
            ctrl_phi_target += ctrl_dot_phi_target*ctrl_Ta

            ctrl_e_phi = ctrl_phi_target - ctrl_phi
            ctrl_dot_e_phi = ctrl_dot_phi_target - ctrl_dot_phi
            if (np.abs(ctrl_dot_psi_K) < dot_psi_K_max):
                ctrl_sum_e_phi = ctrl_sum_e_phi + ctrl_e_phi*ctrl_Ta
            ctrl_dot_psi_K_target = np.clip(KP_ROLL*ctrl_e_phi + KD_ROLL*ctrl_dot_e_phi + KI_ROLL*ctrl_sum_e_phi, -dot_psi_K_max, dot_psi_K_max)

        else:
            ctrl_dotpsi_target = 0
            ctrl_e_psiK = 0 - ctrl_psi_K
            ctrl_dot_e_psiK = 0 - ctrl_dot_psi_K
            if (np.abs(ctrl_phi_target) < np.deg2rad(2)):
                ctrl_sum_e_psiK += ctrl_e_psiK*ctrl_Ta
            ctrl_phi_target_new = -np.clip(KD_TRAJ_BAL*ctrl_dot_e_psiK + KP_TRAJ_BAL*ctrl_e_psiK + KI_TRAJ_BAL*ctrl_sum_e_psiK, np.deg2rad(-2), np.deg2rad(2))
            ctrl_dot_phi_target = (ctrl_phi_target_new - ctrl_phi_target)/ctrl_Ta
            ctrl_phi_target = ctrl_phi_target_new

            ctrl_e_phi = ctrl_phi_target - ctrl_phi
            ctrl_dot_e_phi = ctrl_dot_phi_target - ctrl_dot_phi
            if (np.abs(ctrl_dot_psi_K) < dot_psi_K_max):
                ctrl_sum_e_phi = ctrl_sum_e_phi + ctrl_e_phi*ctrl_Ta
            ctrl_dot_psi_K_target = np.clip(KP_ROLL*ctrl_e_phi + KD_ROLL*ctrl_dot_e_phi + KI_ROLL*ctrl_sum_e_phi, -dot_psi_K_max, dot_psi_K_max)

        ctrl_t += ctrl_Ta

    dot_psi_K_soll[n] = ctrl_dot_psi_K_target
    phi_target[n] = ctrl_phi_target
    dot_psi_target[n] = ctrl_dotpsi_target
    
    # Servo-Verhalten
    ddot_psi_K[n-1] = np.clip((SERVO_K*dot_psi_K_soll[n] - dot_psi_K[n - 1])/SERVO_T, -dot_psi_K_max/delta_t, dot_psi_K_max/delta_t)
    dot_psi_K[n] = dot_psi_K[n-1] + ddot_psi_K[n-1]*delta_t
    psi_K[n] = psi_K[n-1] + dot_psi_K[n-1]*delta_t

    if abs(psi_K[n-1]) >= psi_K_max:
       dot_psi_K[n] = 0
    
    #Störung
    M_Stoerung_Phi[n] = -0.00
    
    # Bewegungssituation
    # Kreiselmomente
    M_kphi[n-1] = MOI_GYRO*w_k*(dot_psi[n-1] + dot_psi_K[n])
    M_kpsi[n-1] = -MOI_GYRO*w_k*dot_phi[n-1]

    # BGL
    # dot_phi mit Gleitreibung
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


rad2deg = 180/np.pi

fig, axs = plt.subplots(3, 2, figsize=(12, 8))
fig.suptitle("Simulation Drive", fontsize=16)

# -------------------- Erste Spalte --------------------
ax = axs[0, 0]
ax.plot(t, phi * rad2deg, label='Phi', color='blue')
ax.plot(t, phi_target * rad2deg, label='PhiTarget', linestyle='--', color='dodgerblue')
ax.set_title("Winkel Rollen")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("Winkel (°)")
ax.grid(True, zorder=0)
ax.legend()

ax = axs[1, 0]
ax.plot(t, dot_phi * rad2deg, label='DotPhi', color='blue')
ax.set_title("Winkelgeschwindigkeit Rollen")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("°/s")
ax.grid(True, zorder=0)
ax.legend()

ax = axs[2, 0]
ax.plot(t, psi_K * rad2deg, label='Position', color='green', zorder=3)
ax.plot(t, dot_psi_K * rad2deg, label='Velocity', color='blue', zorder=2)
ax.plot(t, dot_psi_K_soll * rad2deg, label='TargetVelocity', linestyle='--', color='dodgerblue', zorder=1)
ax.set_title("Gyro-Dynamixel")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("Winkel (°) / °/s")
ax.grid(True)
ax.legend()
ax.set_ylim(-30, 20)

# -------------------- Zweite Spalte --------------------
# --- Geschwindigkeit ---
ax = axs[0, 1]
ax.plot(t, np.full(steps, v), label='Speed', color='black')
ax.set_title("Geschwindigkeit")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("m/s")
ax.grid(True)
ax.legend()

# --- Winkelgeschwindigkeit Gieren ---
ax = axs[1, 1]
ax.plot(t, psi_inert * rad2deg, label='Psi', color='black')
ax.plot(t, dot_psi * rad2deg, label='DotPsi', color='purple')
ax.plot(t, dot_psi_target * rad2deg, label='DotPsiTarget', linestyle='--', color='plum')
ax.set_title("Winkel/Winkelgeschwindigkeit Gieren")
ax.set_xlabel("Zeit (s)")
ax.set_ylabel("(°) / °/s")
ax.grid(True)
ax.legend()

# --- Trajektorie aus Speed & DotPsi ---
ax = axs[2, 1]    
ax.plot(n1, n2, label="Trajektorie", color='black')
ax.set_aspect('equal', 'box')
ax.set_aspect('equal', adjustable='box')

ax.set_aspect('equal', adjustable='box')

xmin, xmax = ax.get_xlim()
ymin, ymax = ax.get_ylim()

# Range von x als Referenz
xrange = xmax - xmin
ymid = 0.5 * (ymin + ymax)

# y-Achse so setzen, dass Range gleich wie x ist und um y-Mitte liegt
ax.set_ylim(ymid - xrange/2, ymid + xrange/2)

ax.set_title("Fahrzeugbahn")
ax.set_xlabel("x (m)")
ax.set_ylabel("y (m)")
ax.grid(True)
ax.legend()

plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()