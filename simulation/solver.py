import numpy as np

class NLSolver:
    def __init__(self, system_dynamics, controller, dt=0.001):
        self.f = system_dynamics
        self.ctrl = controller
        self.dt = dt

    def step(self, x, u, t, *args):
        dx = self.f(x, u, t, *args)
        return x + dx * self.dt

    def simulate(self, x0, duration, callback=None):
        steps = int(duration / self.dt)
        t_vec = np.linspace(0, duration, steps)
        results = np.zeros((steps, len(x0)))
        u_vec = np.zeros(steps)
        
        x = x0
        for i in range(steps):
            t = t_vec[i]
            results[i, :] = x
            u = self.ctrl.update(x, t)
            u_vec[i] = u

            extra_args = (callback(t),) if callback else ()
            x = self.step(x, u, t, *extra_args)
            
        return t_vec, results, u_vec