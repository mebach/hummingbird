import numpy as np
import ballbeamParam as P
import ballbeamParamHW11 as P11

class ballbeamController:
    def __init__(self):
        self.K = P11.K  # state feedback gain
        self.kr = P11.kr  # Input gain
        self.limit = P.Fmax  # Maximum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        z = x.item(0)
        # Construct the state
        x_tilde = x - np.array([[P.ze], [0], [0], [0]])
        zr_tilde = z_r - P.ze

        # equilibrium force
        F_e = P.m1*P.g*(P.ze/P.length) + P.m2*P.g/2.0
        # Compute the state feedback controller
        F_tilde = -self.K @ x_tilde + self.kr * zr_tilde
        F_unsat = F_e + F_tilde
        F = self.saturate(F_unsat.item(0))
        return F

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

