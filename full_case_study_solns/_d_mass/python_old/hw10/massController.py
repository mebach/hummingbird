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

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]

        tau_tilde = self.zCtrl.PID(z_r, z, False)
        tau = self.saturate(tau_tilde)
        return [tau]

    def saturate(self, u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u







