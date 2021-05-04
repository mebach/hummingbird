import numpy as np
import massParamHW13 as P

class massController:
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],
            [0.0],
        ])
        self.force_d1 = 0.0          # control, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # Input gain
        self.L = P.L                 # observer gain
        self.A = P.A                 # system model
        self.B = P.B
        self.C = P.C
        self.limit = P.F_max         # Maxiumum force
        self.Ts = P.Ts               # sample rate of controller

    def update(self, z_r, y):
        # update the observer and extract z_hat
        x_hat = self.update_observer(y)
        z_hat = self.x_hat.item(0)

        # integrate error
        error = z_r - z_hat
        self.integrateError(error)

        # Compute the state feedback controller
        force_tilde = -self.K @ x_hat \
                      - self.ki*self.integrator
        # compute total torque
        force = self.saturate(force_tilde.item(0))
        self.force_d1 = force
        return force, x_hat

    def update_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f(self.x_hat, y_m)
        F2 = self.observer_f(self.x_hat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f(self.x_hat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f(self.x_hat + self.Ts * F3, y_m)
        self.x_hat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        x_hat = np.array([[self.x_hat.item(0)],
                          [self.x_hat.item(1)],
                          ])
        return x_hat

    def observer_f(self, x_hat, y_m):
        # xhatdot = A*xhat + B*(u-ue) + L(y-C*xhat)
        xhat_dot = self.A @ x_hat \
                   + self.B * self.force_d1 \
                   + self.L @ (y_m - self.C @ x_hat)
        return xhat_dot

    def integrateError(self, error):
        self.integrator = self.integrator \
                          + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

