import numpy as np
import ballbeamParam as P
import ballbeamParamHW13 as P13

class ballbeamController:
    def __init__(self):
        self.x_hat = np.array([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0]])  # initial estimate for theta_hat_dot
        self.F_d1 = 0.0  # Computed Force, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P13.K               # state feedback gain
        self.ki = P13.ki             # Integral gain
        self.L = P13.L                 # observer gain
        self.A = P13.A                 # system model
        self.B = P13.B
        self.C = P13.C
        self.limit = P.Fmax          # Maximum force
        self.Ts = P.Ts               # sample rate of controller

    def update(self, z_r, y):
        # update the observer and extract z_hat
        x_hat = self.update_observer(y)
        z_hat = x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrateError(error)

        # Construct the state
        xe = np.array([[P.ze], [0.0], [0.0], [0.0]])
        x_tilde = x_hat - xe

        # equilibrium force
        F_e = P.m1*P.g*P.ze/P.length + P.m2*P.g/2.0
        # Compute the state feedback controller
        F_tilde = -self.K @ x_tilde \
                  - self.ki * self.integrator
        F_unsat = F_e + F_tilde.item(0)
        F = self.saturate(F_unsat)
        self.integratorAntiWindup(F, F_unsat)
        self.F_d1 = F
        return F, x_hat

    def update_observer(self, y):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.x_hat

    def observer_f(self, x_hat, y):
        # xhatdot = A*(xhat-xe) + B*u + L(y-C*xhat)
        xe = np.array([[P.ze], [0.0], [0.0], [0.0]])

        # equilibrium force
        F_e = P.m1*P.g*P.ze/P.length + P.m2*P.g/2.0
        xhat_dot = self.A @ (x_hat - xe) \
                   + self.B * (self.F_d1 - F_e) \
                   + self.L @ (y - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def integratorAntiWindup(self, F, F_unsat):
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator \
                              + P.Ts/self.ki*(F-F_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

