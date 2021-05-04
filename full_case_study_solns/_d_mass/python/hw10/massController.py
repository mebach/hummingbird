import numpy as np
import massParamHW10 as P
import sys
sys.path.append('..')  # add parent directory
import massParam as P0
from PIDControl import PIDControl

class massController:
    def __init__(self):
        # Instantiates the PD object
        self.zCtrl = PIDControl(P.kp, P.ki, P.kd, P0.F_max, P0.beta, P0.Ts)
        self.limit = P0.F_max

    def update(self, z_r, y):
        z = y.item(0)
        force_tilde = self.zCtrl.PID(z_r, z, False)
        force = self.saturate(force_tilde)
        return force

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







