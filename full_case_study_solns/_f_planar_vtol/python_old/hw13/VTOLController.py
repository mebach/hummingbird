import numpy as np
import VTOLParam as P
import VTOLParamHW13 as P13

class VTOLController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.xhat_lon = np.matrix([[0.0], [0.0]])
        self.xhat_lat = np.matrix([[0.0], [0.0], [0.0], [0.0]])
        self.xhat = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.integrator_z = 0.0      # integrator on position z
        self.error_z_d1 = 0.0        # error signal delayed by 1 sample
        self.integrator_h = 0.0      # integrator on altitude h
        self.error_h_d1 = 0.0        # error signal delayed by 1 sample
        self.F_d1 = 0.0  # Force signal delayed by 1 sample
        self.tau_d1 = 0.0  # torque signal delayed by 1 sample
        self.limit = P.fmax

    def u(self, r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = float(r[0])
        h_r = float(r[1])
        z = y[0]
        h= y[1]
        theta = y[2]

        # update the observers
        self.updateObserver(y)
        z_hat = self.xhat_lat[0]
        h_hat = self.xhat_lon[0]

        # integrate error
        error_z = z_r - z_hat
        self.integrateErrorZ(error_z)
        error_h = h_r - h_hat
        self.integrateErrorH(error_h)

        # Construct the states
        # Compute the state feedback controllers
        F_tilde = -P13.K_lon*self.xhat_lon - P13.ki_lon*self.integrator_h
        F = P.Fe/np.cos(theta) + F_tilde
        tau = -P13.K_lat*self.xhat_lat - P13.ki_lat*self.integrator_z
        u = np.concatenate((F, tau), axis=0)
        self.updateForce(F)
        self.updateTorque(tau)
        self.x_hat = [self.xhat_lat.item(0),
                     self.xhat_lon.item(0),
                     self.xhat_lat.item(1),
                     self.xhat_lat.item(2),
                     self.xhat_lon.item(1),
                     self.xhat_lat.item(3)]
        return u

    def updateObserver(self, y_m):
        N = 10
        y_lon = np.matrix([[y_m[1]]])
        y_lat = np.matrix([
                [y_m[0]],
                [y_m[2]]])
        for i in range(0, N):
            self.xhat_lon = self.xhat_lon + P.Ts/float(N)*(
                P13.A_lon*self.xhat_lon
                + P13.B_lon*(self.F_d1 - P.Fe)
                + P13.L_lon*(y_lon-P13.C_lon*self.xhat_lon))
            self.xhat_lat = self.xhat_lat + P.Ts/float(N)*(
                P13.A_lat*self.xhat_lat
                + P13.B_lat*self.tau_d1
                + P13.L_lat*(y_lat-P13.C_lat*self.xhat_lat))

    def updateForce(self, F):
        self.F_d1 = F

    def updateTorque(self, tau):
        self.tau_d1 = tau

    def integrateErrorZ(self, error_z):
        self.integrator_z = self.integrator_z + (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z

    def integrateErrorH(self, error_h):
        self.integrator_h = self.integrator_h + (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

