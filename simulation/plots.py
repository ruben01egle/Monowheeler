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
    axs[0, 1].plot(t_vec, x_vec[:,3], label='Position', color='red')
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