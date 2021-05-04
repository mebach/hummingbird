import sys
sys.path.append('..')  # add parent directory
import massParam as P
import loopshape_mass as L

import numpy as np

class massController:
    def __init__(self):
        self.x_C = np.zeros((L.C_ss.A.shape[0], 1))
        self.x_F = np.zeros((L.F_ss.A.shape[0], 1))
        self.limit = P.F_max  # Maximum force
        self.N = 10  #number of Euler integration steps for each sample

    def update(self, z_r, y_m):


        # prefilter
        self.updatePrefilterState(z_r)
        z_r_filtered = L.F_ss.C * self.x_F + L.F_ss.D * z_r
        # error signal for longitudinal loop
        error = z_r_filtered - y_m
        # longitudinal controller
        self.updateControlState(error)
        F = L.C_ss.C * self.x_C + L.C_ss.D * error
        # equilibrium force
        # force = self.saturate(F.item(0))

        return F.item(0)

    def updatePrefilterState(self, z_r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (P.Ts/self.N)*(
                L.F_ss.A*self.x_F + L.F_ss.B*z_r
            )

    def updateControlState(self, error):
        for i in range(0, self.N):
            self.x_C = self.x_C + (P.Ts/self.N)*(
                L.C_ss.A*self.x_C + L.C_ss.B*error
            )


    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

