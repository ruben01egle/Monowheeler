import numpy as np

class MonowheelerConfig:
    def __init__(self):
        # --- physical paramaters ---
        self.MASS_PLATFORM = 10.971  # kg
        self.MASS_TOTAL = 13.971     # kg
        self.G = 9.81                # m/s^2
        self.H_0 = 0.1123            # m (hight cog)
        
        # --- Trägheitsmomente (MOI) ---
        self.MOI_XX_TAU = 0.2489     # kg*m^2 (Roll)
        self.MOI_YY_TAU = 0.6826     # kg*m^2 (Pitch)
        self.MOI_ZZ = 0.5017         # kg*m^2 (Yaw)
        
        # --- gyro ---
        self.N_K = 5000              # RPM
        self.MOI_GYRO = 3.3991489e-03 # kg*m^2
        
        # --- Dynamik & Environment ---
        self.FRICTION_ROLL_COEFF = 200
        self.FRICTION_YAW_COEFF = 0.01
        self.STATIC_FRICTION_YAW_M_THRESH = 0.03
        self.STATIC_FRICTION_YAW_DOT_PHI_THRESH = 0.01
        
        # --- Acuators PT1 ---
        self.SERVO_PITCH_K = 0.85
        self.SERVO_PITCH_T = 0.045
        self.SERVO_ROLL_K = 0.85
        self.SERVO_ROLL_T = 0.10
        self.WHEEL_K = 1.0
        self.WHEEL_T = 0.1

        # --- limits ---
        self.MAX_THETA = 0.175
        self.MAX_PHI = 0.25
        self.MAX_P = 0.032
        self.MAX_DOT_P = 0.2
        self.MAX_PSI_K = 0.25
        self.MAX_DOT_PSI_K = 5

    @property
    def W_K(self):
        return self.N_K / 60 * 2 * np.pi

    @property
    def F_GP(self):
        return self.G * self.MASS_PLATFORM

    @property
    def F_G(self):
        return self.G * self.MASS_TOTAL



class MonowheelerPitchPhysics:
    def __init__(self, config):
        self.cfg = config
    
    def dynamics(self, x, u, t, dist=0.0):
        # x = [theta, dot_theta, p]
        theta, dot_theta, p = x
        p_cmd = u
        M_dist = dist

        dot_p = np.clip((self.cfg.SERVO_PITCH_K*p_cmd - p)/self.cfg.SERVO_PITCH_T, -self.cfg.MAX_DOT_P, self.cfg.MAX_DOT_P)

        M_wheelshift = self.cfg.MASS_PLATFORM * self.cfg.G * p
        M_lean = self.cfg.MASS_TOTAL * self.cfg.G * self.cfg.H_0 * theta
        ddot_theta = (M_wheelshift + M_dist + M_lean) / self.cfg.MOI_YY_TAU

        if theta >= self.cfg.MAX_THETA and ddot_theta > 0:
            ddot_theta = 0
            dot_theta = 0
        elif theta <= -self.cfg.MAX_THETA and ddot_theta < 0:
            ddot_theta = 0
            dot_theta = 0

        if p >= self.cfg.MAX_P and dot_p > 0:
            dot_p = 0
        elif p <= -self.cfg.MAX_P and dot_p < 0:
            dot_p = 0
        
        return np.array([dot_theta, ddot_theta, dot_p])
    
class MonowheelerRollYawPhysics:
    def __init__(self, config):
        self.cfg = config

    def dynamics(self, x, u, t, dist=0.0):
        # x = [phi, dot_phi, dot_psi, psi_k, dot_psi_k, v]
        phi, dot_phi, dot_psi, psi_k, dot_psi_k, v = x
        dot_psi_k_cmd, _, _, v_cmd = u
        M_dist = dist

        friction_roll = self.cfg.FRICTION_ROLL_COEFF*(1.0 + 1.0/(abs(v)+0.1))
        static_friction_M_thresh = self.cfg.STATIC_FRICTION_YAW_M_THRESH*(1.0 + 1.0/(abs(v)+0.1))
        static_friction_dotphi_thresh = self.cfg.STATIC_FRICTION_YAW_DOT_PHI_THRESH*(1.0 + 1.0/(abs(v)+0.1))
        
        ddot_psi_k = (self.cfg.SERVO_ROLL_K * dot_psi_k_cmd - dot_psi_k) / self.cfg.SERVO_ROLL_T

        M_k_phi = self.cfg.MOI_GYRO*self.cfg.W_K*(dot_psi + dot_psi_k)
        M_k_psi = -self.cfg.MOI_GYRO*self.cfg.W_K*dot_phi

        ddot_phi = 1/self.cfg.MOI_XX_TAU * (M_k_phi + self.cfg.MASS_TOTAL*v*dot_psi*self.cfg.H_0 + self.cfg.F_G*np.sin(phi)*self.cfg.H_0 + M_dist) - np.sign(dot_phi)*friction_roll*dot_phi**2

        M_psi = M_k_psi - np.sign(dot_psi)*self.cfg.FRICTION_YAW_COEFF*dot_psi**2
        if abs(M_psi) < static_friction_M_thresh  and abs(dot_psi) < static_friction_dotphi_thresh:
            friction_stick = 100
            ddot_psi = -friction_stick*dot_psi
        else:
            ddot_psi = 1/self.cfg.MOI_ZZ*M_psi

        dot_v = (self.cfg.WHEEL_K * v_cmd - v) / self.cfg.WHEEL_T

        if psi_k >= self.cfg.MAX_PSI_K and dot_psi_k > 0:
            dot_psi_k = 0
            if ddot_psi_k > 0: ddot_psi_k = 0
        elif psi_k <= -self.cfg.MAX_PSI_K and dot_psi_k < 0:
            dot_psi_k = 0
            if ddot_psi_k < 0: ddot_psi_k = 0
    
        if dot_psi_k >= self.cfg.MAX_DOT_PSI_K and ddot_psi_k > 0:
            ddot_psi_k = 0
        elif dot_psi_k <= -self.cfg.MAX_DOT_PSI_K and ddot_psi_k < 0:
            ddot_psi_k = 0

        if phi >= self.cfg.MAX_PHI and ddot_phi > 0:
            ddot_phi = 0
            dot_phi = 0
        elif phi <= -self.cfg.MAX_PHI and ddot_phi < 0:
            ddot_phi = 0
            dot_phi = 0

        return np.array([dot_phi, ddot_phi, ddot_psi, dot_psi_k, ddot_psi_k, dot_v])