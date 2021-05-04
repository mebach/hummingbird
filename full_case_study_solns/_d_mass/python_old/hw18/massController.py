import sys
sys.path.append('..')  # add parent directory
import massParam as P
import loopshape_mass as L
import numpy as np

class massController:
    # state feedback control using dirty derivatives to estimate zdot
    def __init__(self):
        self.x_C = np.zeros((L.C_ss.A.shape[0], 1))
        self.x_F = np.zeros((L.F_ss.A.shape[0], 1))
        self.A_F = L.F_ss.A
        self.B_F = L.F_ss.B
        self.C_F = L.F_ss.C
        self.D_F = L.F_ss.D
        self.A_C = L.C_ss.A
        self.B_C = L.C_ss.B
        self.C_C = L.C_ss.C
        self.D_C = L.C_ss.D
        self.limit = P.F_max  # Maximum force
        self.Ts = P.Ts  # sample rate of controller
        self.N = 10  # number of Euler integration steps for each sample

    def u(self, y_r, y_m):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y_m[0]

         # solve differential equation defining prefilter F
        self.updatePrefilterState(z_r)
        z_r_filtered = self.C_F * self.x_F + self.D_F * z_r

        # filtered error signal
        error = z_r_filtered - z

        # solve differential equation defining control C
        self.updateControlState(error)
        force_tilde = self.C_C * self.x_C + self.D_C * error

        # compute total torque
        force = self.saturate(force_tilde)
        return [force.item(0)]

    def updatePrefilterState(self, z_r):
        for i in range(0, self.N):
            self.x_F = self.x_F + (self.Ts/self.N)*(
                self.A_F*self.x_F + self.B_F*z_r
            )

    def updateControlState(self, error):
        for i in range(0, self.N):
            self.x_C = self.x_C + (self.Ts/self.N)*(
                self.A_C*self.x_C + self.B_C*error
            )

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

