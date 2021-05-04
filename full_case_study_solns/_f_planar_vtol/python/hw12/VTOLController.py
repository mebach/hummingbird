import numpy as np
import VTOLParam as P
import VTOLParamHW12 as P12

class VTOLController:
    def __init__(self):
        self.integrator_z = 0.0  # integrator on position z
        self.error_z_d1 = 0.0  # error signal delayed by 1 sample
        self.integrator_h = 0.0  # integrator on altitude h
        self.error_h_d1 = 0.0  # error signal delayed by 1 sample
        self.limit = P.fmax

    def update(self, r, x):
        z_r = r.item(0)
        h_r = r.item(1)
        z = x.item(0)
        h = x.item(1)
        theta = x.item(2)
        # integrate error
        error_z = z_r - z
        self.integrateErrorZ(error_z)
        error_h = h_r - h
        self.integrateErrorH(error_h)

        # Construct the states
        x_lon = np.array([[x.item(1)], [x.item(4)]])
        x_lat = np.array([[x.item(0)], [x.item(2)], [x.item(3)], [x.item(5)]])
        # Compute the state feedback controllers
        F_tilde = -P12.K_lon @ x_lon - P12.ki_lon * self.integrator_h
        F = P.Fe/np.cos(theta) + F_tilde.item(0)
        tau = -P12.K_lat @ x_lat - P12.ki_lat*self.integrator_z
        return np.array([[F], [tau.item(0)]])

    def differentiateZ(self, z):
        self.z_dot = P.beta*self.z_dot + (1-P.beta)*((z - self.z_d1) / P.Ts)
        self.z_d1 = z

    def differentiateH(self, h):
        self.h_dot = P.beta*self.h_dot + (1-P.beta)*((h - self.h_d1) / P.Ts)
        self.h_d1 = h

    def differentiateTheta(self, theta):
        self.theta_dot = P.beta*self.theta_dot + (1-P.beta)*((theta - self.theta_d1) / P.Ts)
        self.theta_d1 = theta

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

