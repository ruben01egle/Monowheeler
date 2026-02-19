from monowheeler_physics import MonowheelerConfig, MonowheelerRollYawPhysics
from monowheeler_controll import RollYawController, RollControllerConfig, YawControllerConfig
from solver import NLSolver
from plots import roll_yaw_plot

import numpy as np

dt = 0.0001
Ta = 1/250
T_end = 15.0

def corner_generator(t):
    dot_psi_target = 0
    if t > 1 and t < 5:
        dot_psi_target = -0.15
    return dot_psi_target

# 1. Setup
physics_cfg = MonowheelerConfig()
ctrl_cfg_roll = RollControllerConfig(Ta)
ctrl_cfg_yaw = YawControllerConfig(Ta, corner_generator)
model = MonowheelerRollYawPhysics(physics_cfg)
controller = RollYawController(ctrl_cfg_roll, ctrl_cfg_yaw)
solver = NLSolver(model.dynamics, controller, dt)

# Initialisierung Zustandsvektor: [phi, dot_phi, psi, dot_psi, psi_k, dot_psi_k, v]
x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, physics_cfg.V])

t_vec, x_vec, u_vec  = solver.simulate(x0, T_end)

roll_yaw_plot(t_vec, x_vec, u_vec)