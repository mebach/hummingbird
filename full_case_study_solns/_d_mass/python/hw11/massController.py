import numpy as np
import massParamHW11 as P

class massController:
    def __init__(self):
        self.K = P.K  # state feedback gain
        self.kr = P.kr  # Input gain
        self.limit = P.F_max  # Maxiumum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        # Compute the state feedback controller
        force_tilde = -self.K*x + self.kr*z_r
        # compute total torque
        force = self.saturate(force_tilde.item(0))
        return force

    def differentiateZ(self, z):
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

