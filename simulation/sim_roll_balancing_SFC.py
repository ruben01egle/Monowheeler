from framework.monowheeler_physics import MonowheelerConfig, MonowheelerRollYawPhysics
from framework.monowheeler_controll import RollYawController, RollControllerConfig
from framework.solver import NLSolver
from framework.plots import roll_plot

import numpy as np

dt = 0.0001
Ta = 1/250
T_end = 30.0

# 1. Setup
physics_cfg = MonowheelerConfig()
ctrl_cfg = RollControllerConfig(Ta)
model = MonowheelerRollYawPhysics(physics_cfg)
controller = RollYawController(ctrl_cfg)
solver = NLSolver(model.dynamics, controller, dt)

# Initialisierung Zustandsvektor: [phi, dot_phi, psi, dot_psi, psi_k, Int(psi_k), dot_psi_k, v]
x0 = np.array([np.deg2rad(1), 0.0, 0.0, 0.0, 0.0, 0.0])

def disturbance(t):
    if t > 15 and t < 15.5:
        dist = -0.05
    else:
        dist = 0.05
    return dist 

t_vec, x_vec, u_vec  = solver.simulate(x0, T_end, disturbance)

roll_plot(t_vec, x_vec, u_vec)