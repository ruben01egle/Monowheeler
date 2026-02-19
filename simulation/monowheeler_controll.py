import numpy as np

class PitchControllerConfig:
    def __init__(self, TA):
        self.TA = TA
        self.Kd = np.array([0.66638156,  0.12917211, -0.28520978,  1.02820138])
        self.MAX_P = 0.032


class PitchController:
    def __init__(self, config):
        self.cfg = config
        self.last_sync_t = -0.001
        self.sum_theta = 0
        self.p_cmd = 0.0

    def update(self, x, t):
        
        if t - self.last_sync_t < self.cfg.TA:
            return self.p_cmd

        theta, dot_theta, p = x
        if abs(self.p_cmd) < self.cfg.MAX_P:
            self.sum_theta -= theta*self.cfg.TA

        u = -(self.cfg.Kd[0]*theta + self.cfg.Kd[1]*dot_theta + 
                   self.cfg.Kd[2]*self.sum_theta + self.cfg.Kd[3]*p)

        self.p_cmd = np.clip(u, -self.cfg.MAX_P, self.cfg.MAX_P)
        
        self.last_sync_t = t
        return self.p_cmd
    
class RollControllerConfig:
    def __init__(self, TA):
        self.TA = TA
        self.Kd = np.array([25, 4, -1.8, -0.15, 0.3])
        self.MAX_DOT_PSI_K = 5

class YawControllerConfig:
    def __init__(self, TA, callback):
        self.TA = TA
        self.roll_pid = np.array([15.0, 3.0, 3.0])
        self.yaw_pid = np.array([1.2, 0.2, 1.5])
        self.MAX_DOT_PSI = 0.09
        self.MAX_DOT_PSI_K = 5
        self.callback = callback
    
class RollYawController():
    def __init__(self, config_roll, config_yaw=None):
        self.cfg_roll = config_roll
        self.last_sync_t = -0.001
        self.sum_psi_k = 0
        self.phi_target = 0
        self.dot_psi_target = 0
        self.dot_psi_k_cmd = 0.0
        self.cfg_yaw = config_yaw
        if self.cfg_yaw:
            self.pid_roll = PID(self.cfg_yaw.roll_pid, self.cfg_yaw.TA, self.cfg_yaw.MAX_DOT_PSI_K)
            self.pid_yaw = PID(self.cfg_yaw.yaw_pid, self.cfg_yaw.TA, self.cfg_yaw.MAX_DOT_PSI)

    def update(self, x, t):
        if t - self.last_sync_t < self.cfg_roll.TA:
            return np.array([self.dot_psi_k_cmd, self.phi_target, self.dot_psi_target])

        phi, dot_phi, dot_psi, psi_k, dot_psi_k, v = x
        if self.cfg_yaw:
            self.dot_psi_target = self.cfg_yaw.callback(t)
        
        if self.dot_psi_target != 0:
            dot_phi_target = -self.pid_yaw.control(self.dot_psi_target, dot_psi)
            self.phi_target += dot_phi_target*self.cfg_yaw.TA
            u = self.pid_roll.control(self.phi_target, phi, dot_phi_target, dot_phi)
        else:
            self.phi_target = 0
            self.sum_psi_k += psi_k*self.cfg_roll.TA
            u = -(self.cfg_roll.Kd[0]*phi + self.cfg_roll.Kd[1]*dot_phi + 
                self.cfg_roll.Kd[2]*psi_k + self.cfg_roll.Kd[3]*self.sum_psi_k +
                self.cfg_roll.Kd[4]*dot_psi_k)
        
        self.dot_psi_k_cmd = np.clip(u, -self.cfg_roll.MAX_DOT_PSI_K, self.cfg_roll.MAX_DOT_PSI_K)
        
        self.last_sync_t = t
        return np.array([self.dot_psi_k_cmd, self.phi_target, self.dot_psi_target])
    
class PID:
    def __init__(self, K, TA, MAX_U):
        self.K = K
        self.TA = TA
        self.MAX_U = MAX_U
        self.e = 0.0
        self.e_prev = 0.0
        self.dot_e = 0.0
        self.sum_e = 0.0
        self.u = 0.0

    def control(self, w, x, dot_w=None, dot_x=None):
        self.e = w - x
        
        if dot_w is not None and dot_x is not None:
            self.dot_e = dot_w - dot_x
        else:
            self.dot_e = (self.e - self.e_prev) / self.TA
            
        if abs(self.u) < self.MAX_U:
            self.sum_e += self.e * self.TA
            
        u_unclamped =(
            self.K[0] * self.e + 
            self.K[1] * self.dot_e + 
            self.K[2] * self.sum_e
        )
        
        self.u = np.clip(u_unclamped, -self.MAX_U, self.MAX_U)
        self.e_prev = self.e
        return self.u

    def reset(self):
        self.e = 0.0
        self.e_prev = 0.0
        self.dot_e = 0.0
        self.sum_e = 0.0
        self.u = 0.0