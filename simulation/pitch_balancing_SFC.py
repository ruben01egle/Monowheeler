from monowheeler_physics import MonowheelerConfig, MonowheelerPitchPhysics
from monowheeler_controll import PitchController, PitchControllerConfig
from solver import NLSolver
from plots import pitch_plot

import numpy as np

dt = 0.0001
Ta = 1/250
T_end = 15.0

# 1. Setup
physics_cfg = MonowheelerConfig()
ctrl_cfg = PitchControllerConfig(Ta)
model = MonowheelerPitchPhysics(physics_cfg)
controller = PitchController(ctrl_cfg)
solver = NLSolver(model.dynamics, controller, dt)

# Initialisierung Zustandsvektor: [theta, dot_theta, sum_theta, p]
x0 = np.array([np.deg2rad(5.0), 0.0, 0.0, 0.0])

def disturbance(t):
    if t > 5 and t < 5.1:
        dist = 1
    elif t > 9:
        dist = 0.2
    else:
        dist = 0
    return dist 

t_vec, x_vec, u_vec  = solver.simulate(x0, T_end, disturbance)

pitch_plot(t_vec, x_vec, u_vec)