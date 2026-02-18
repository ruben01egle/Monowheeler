import numpy as np
from enum import Enum

class pitch_mode(Enum):
    SLOW = 1
    FAST = 2
    T_S2F = 3
    T_F2S = 4

class PitchControllerConfig:
    def __init__(self, TA):
        self.TA = TA
        self.Kd_fast = np.array([0.66638156,  0.12917211, -0.28520978,  1.02820138])
        self.Kd_slow = np.array([0.45*self.Kd_fast[0],  0.5*self.Kd_fast[1], self.Kd_fast[2],  self.Kd_fast[3]])
        self.MAX_P = 0.032
        self.THETA_UPPER = 0.015
        self.THETA_LOWER = 0.005
        self.DOT_THETA_UPPER = 0.0375
        self.DOT_THETA_LOWER = 0.009
        self.T_TRANSITION_MAX = 10*self.TA
        self.T_SLOW_THRESHOLD = 0.25


class PitchController:
    def __init__(self, config):
        self.cfg = config
        self.last_sync_t = -0.001
        
        self.mode = pitch_mode.SLOW
        self.t_transition = 0.0
        self.t_slow_accumulator = 0.0
        self.p_cmd = 0.0

    def update(self, x, t):
        
        if t - self.last_sync_t < self.cfg.TA:
            return self.p_cmd

        theta, dot_theta, sum_theta, p = x

        if self.mode == pitch_mode.SLOW:
            if abs(theta) > self.cfg.THETA_UPPER or abs(dot_theta) > self.cfg.DOT_THETA_UPPER:
                self.mode = pitch_mode.T_S2F
                self.t_transition = 0.0
        
        elif self.mode == pitch_mode.FAST:
            if abs(theta) < self.cfg.THETA_LOWER and abs(dot_theta) < self.cfg.DOT_THETA_LOWER:
                self.t_slow_accumulator += self.cfg.TA
                if self.t_slow_accumulator > self.cfg.T_SLOW_THRESHOLD:
                    self.mode = pitch_mode.T_F2S
                    self.t_transition = 0.0
                    self.t_slow_accumulator = 0.0
            else:
                self.t_slow_accumulator = 0.0

        elif self.mode in [pitch_mode.T_S2F, pitch_mode.T_F2S]:
            self.t_transition += self.cfg.TA
            if self.t_transition > self.cfg.T_TRANSITION_MAX:
                self.mode = pitch_mode.FAST if self.mode == pitch_mode.T_S2F else pitch_mode.SLOW

        u_fast = -(self.cfg.Kd_fast[0]*theta + self.cfg.Kd_fast[1]*dot_theta + 
                   self.cfg.Kd_fast[2]*sum_theta + self.cfg.Kd_fast[3]*p)
        
        u_slow = -(self.cfg.Kd_slow[0]*theta + self.cfg.Kd_slow[1]*dot_theta + 
                   self.cfg.Kd_slow[2]*sum_theta + self.cfg.Kd_slow[3]*p)

        if self.mode == pitch_mode.FAST:
            self.p_cmd = u_fast
        elif self.mode == pitch_mode.SLOW:
            self.p_cmd = u_slow
        elif self.mode == pitch_mode.T_S2F:
            blend = np.clip(self.t_transition / self.cfg.T_TRANSITION_MAX, 0.0, 1.0)
            self.p_cmd = blend * u_fast + (1 - blend) * u_slow
        elif self.mode == pitch_mode.T_F2S:
            blend = np.clip(self.t_transition / self.cfg.T_TRANSITION_MAX, 0.0, 1.0)
            self.p_cmd = blend * u_slow + (1 - blend) * u_fast

        self.p_cmd = np.clip(self.p_cmd, -self.cfg.MAX_P, self.cfg.MAX_P)
        
        self.last_sync_t = t
        return self.p_cmd
    
class RollBalanceController:
    def __init__(self, config):
        self.cfg = config
        self.last_sync_t = -0.001
        self.dot_psi_k_cmd = 0.0

    def update(self, x, t):
        
        if t - self.last_sync_t < self.cfg.TA:
            return self.dot_psi_k_cmd

        phi, dot_phi, psi, dot_psi, psi_k, sum_psi_k, dot_psi_k, v = x

        u = -(self.cfg.Kd[0]*phi + self.cfg.Kd[1]*dot_phi + 
              self.cfg.Kd[2]*psi_k + self.cfg.Kd[3]*sum_psi_k +
              self.cfg.Kd[4]*dot_psi_k)
        
        self.dot_psi_k_cmd = np.clip(u, -self.cfg.MAX_DOT_PSI_K, self.cfg.MAX_DOT_PSI_K)
        
        self.last_sync_t = t
        return self.dot_psi_k_cmd