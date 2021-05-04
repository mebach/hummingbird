import numpy as np
import massParamHW8 as P
import sys
sys.path.append('..')  # add parent directory
import massParam as P0


class massController:

    def __init__(self):
        self.kp = P.kp
        self.kd = P.kd
        self.F_max = P0.F_max

    def update(self, z_r, state):
        z = state.item(0)
        zdot = state.item(1)

        tau_tilde = self.kp * (z_r - z) - self.kd * zdot
        tau = self.saturate(tau_tilde, self.F_max)
        return tau

    def saturate(self, u, limit):
        if abs(u) > limit:
            u = limit * np.sign(u)
        return u







