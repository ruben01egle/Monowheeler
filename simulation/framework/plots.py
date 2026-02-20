import matplotlib.pyplot as plt
import numpy as np

def pitch_plot(t_vec, x_vec, u_vec):
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Simulation Pitch", fontsize=16)

    # oberer linker Plot: theta und w
    axs[0, 0].plot(t_vec, np.rad2deg(x_vec[:,0]), label='Theta', color='red')
    axs[0, 0].plot(t_vec, np.zeros_like(t_vec), label='ThetaTarget', linestyle='--',
                    color='lightcoral')
    axs[0, 0].set_title("Winkel Nicken")
    axs[0, 0].set_xlabel("Zeit (s)")
    axs[0, 0].set_ylabel("Winkel (°)")
    axs[0, 0].legend()
    axs[0, 0].grid()

    # oberer rechter Plot: p und p_soll
    axs[0, 1].plot(t_vec, u_vec, label='TargetPosition',
                    linestyle='--', color='lightcoral')
    axs[0, 1].plot(t_vec, x_vec[:,2], label='Position', color='red')
    axs[0, 1].set_title("Wheel-Dynamixel Radverschiebung")
    axs[0, 1].set_xlabel("Zeit (s)")
    axs[0, 1].set_ylabel("m")
    axs[0, 1].legend()
    axs[0, 1].grid()

    # unterer linker Plot: Winkelgeschwindigkeit dot_theta
    axs[1, 0].plot(t_vec, np.rad2deg(x_vec[:,1]), label='DotTheta', color='red')
    axs[1, 0].set_title("Winkelgeschwindigkeit Nicken")
    axs[1, 0].set_xlabel('t in s')
    axs[1, 0].set_ylabel('°/s')
    axs[1, 0].legend()
    axs[1, 0].grid()

    # unterer rechter Plot leer lassen oder für spätere Nutzung
    axs[1, 1].axis('off')

    plt.tight_layout()
    plt.show()

def roll_plot(t_vec, x_vec, u_vec):
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Simulation Roll", fontsize=16)

    # --- Plot 1: Rollwinkel ---
    ax = axs[0, 0]
    ax.plot(t_vec, np.rad2deg(x_vec[:,0]), label='Phi', color='blue')
    ax.plot(t_vec, np.zeros_like(t_vec), label='PhiTarget', linestyle='--', color='dodgerblue')
    ax.set_title("Winkel Rollen")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("Winkel (°)")
    ax.grid(True)
    ax.legend()

    # --- Plot 2: Gyro-Dynamixel (psi_K) ---
    ax = axs[0, 1]
    ax.plot(t_vec, np.rad2deg(x_vec[:,3]), label='Position', color='green', zorder=3)
    ax.plot(t_vec, np.rad2deg(x_vec[:,4]), label='Velocity', color='blue', zorder=2)
    ax.plot(t_vec, np.rad2deg(u_vec[:,0]), label='TargetVelocity', linestyle='--', color='dodgerblue', zorder=1)
    ax.set_title("Gyro-Dynamixel")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("Winkel (°) / °/s")
    ax.grid(True)
    ax.legend()

    # --- Plot 3: Winkelgeschwindigkeit des Rollens ---
    ax = axs[1, 0]
    ax.plot(t_vec, np.rad2deg(x_vec[:,1]), label='DotPhi', color='blue')
    ax.set_title("Winkelgeschwindigkeit Rollen")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("°/s")
    ax.grid(True)
    ax.legend()

    # --- Plot 4: Gierwinkel ---
    ax = axs[1, 1]
    ax.plot(t_vec, np.rad2deg(x_vec[:,2]), label='DotPsi', color='purple')
    ax.plot(t_vec, np.rad2deg(u_vec[:,2]), label='DotPsiTarget', linestyle='--', color='plum')
    ax.set_title("Winkelgeschwindigkeit Gieren")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("°/s")
    ax.grid(True)
    ax.legend()

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

def roll_yaw_plot(t_vec, x_vec, u_vec):
    # --- Parameter extrahieren ---
    delta_t = t_vec[1] - t_vec[0]
    phi = x_vec[:, 0]
    dot_psi = x_vec[:, 2]
    v = x_vec[:, 5]

    # --- 1. Psi_inert berechnen ---
    # dot_psi_inert = dot_psi * cos(phi)
    dot_psi_inert = dot_psi * np.cos(phi)
    
    # Integration: psi_inert = sum(dot_psi_inert * delta_t)
    # np.cumsum berechnet das laufende Integral
    psi_inert = np.cumsum(dot_psi_inert) * delta_t

    # --- 2. Trajektorie (n1, n2) berechnen ---
    # n1 = sum(v * cos(psi_inert) * delta_t)
    # n2 = sum(v * sin(psi_inert) * delta_t)
    n1 = np.cumsum(v * np.cos(psi_inert)) * delta_t
    n2 = np.cumsum(v * np.sin(psi_inert)) * delta_t


    fig, axs = plt.subplots(3, 2, figsize=(12, 8))
    fig.suptitle("Simulation Drive", fontsize=16)

    # -------------------- Erste Spalte --------------------
    ax = axs[0, 0]
    ax.plot(t_vec, np.rad2deg(x_vec[:,0]), label='Phi', color='blue')
    ax.plot(t_vec, np.rad2deg(u_vec[:,1]), label='PhiTarget', linestyle='--', color='dodgerblue')
    ax.set_title("Winkel Rollen")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("Winkel (°)")
    ax.grid(True, zorder=0)
    ax.legend()

    ax = axs[1, 0]
    ax.plot(t_vec, np.rad2deg(x_vec[:,1]), label='DotPhi', color='blue')
    ax.set_title("Winkelgeschwindigkeit Rollen")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("°/s")
    ax.grid(True, zorder=0)
    ax.legend()

    ax = axs[2, 0]
    ax.plot(t_vec, np.rad2deg(x_vec[:,3]), label='Position', color='green', zorder=3)
    ax.plot(t_vec, np.rad2deg(x_vec[:,4]), label='Velocity', color='blue', zorder=2)
    ax.plot(t_vec, np.rad2deg(u_vec[:,0]), label='TargetVelocity', linestyle='--', color='dodgerblue', zorder=1)
    ax.set_title("Gyro-Dynamixel")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("Winkel (°) / °/s")
    ax.grid(True)
    ax.legend()
    ax.set_ylim(-30, 20)

    # -------------------- Zweite Spalte --------------------
    # --- Geschwindigkeit ---
    ax = axs[0, 1]
    ax.plot(t_vec, x_vec[:,5], label='Speed', color='black')
    ax.set_title("Geschwindigkeit")
    ax.set_xlabel("Zeit (s)")
    ax.set_ylabel("m/s")
    ax.grid(True)
    ax.legend()

    # --- Winkelgeschwindigkeit Gieren ---
    ax = axs[1, 1]
    ax.plot(t_vec, np.rad2deg(psi_inert), label='Psi', color='black')
    ax.plot(t_vec, np.rad2deg(x_vec[:,2]), label='DotPsi', color='purple')
    ax.plot(t_vec, np.rad2deg(u_vec[:,2]), label='DotPsiTarget', linestyle='--', color='plum')
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