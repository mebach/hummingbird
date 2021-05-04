import numpy as np
import ballbeamParam as P
import ballbeamParamHW12 as P12

class ballbeamController:
    def __init__(self):
        self.integrator = 0.0  # integrator
        self.error_d1 = 0.0  # error signal delayed by 1 sample
        self.K = P12.K  # state feedback gain
        self.ki = P12.ki  # Integral gain
        self.limit = P.Fmax  # Maximum force
        self.Ts = P.Ts  # sample rate of controller

    def update(self, z_r, x):
        z = x.item(0)
        # integrate error
        error = z_r - z
        self.integrateError(error)
        # Construct the linearized state
        x_tilde = x - np.array([[P.ze], [0], [0], [0]])
        zr_tilde = z_r - P.ze

        # equilibrium force
        F_e = P.m1*P.g*P.ze/P.length + P.m2*P.g/2.0
        # Compute the state feedback controller
        F_tilde = -self.K @ x_tilde - self.ki*self.integrator
        F_unsat = F_e + F_tilde
        F = self.saturate(F_unsat.item(0))
        self.integratorAntiWindup(F, F_unsat)
        return F

    def differentiateZ(self, z):
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def differentiateTheta(self, theta):
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def integratorAntiWindup(self, F, F_unsat):
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts/self.ki*(F-F_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

