import numpy as np
import massParamHW12 as P

class massController:
    # state feedback control using dirty derivatives to estimate zdot
    def __init__(self):
        self.z_dot = 0.0             # derivative of z
        self.z_d1 = 0.0              # z delayed by 1 sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P.K                 # state feedback gain
        self.ki = P.ki               # Input gain
        self.limit = P.F_max       # Maxiumum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]

        # differentiate z
        self.differentiateZ(z)

        # integrate error
        error = z_r - z
        self.integrateError(error)

        # Construct the state
        x = np.matrix([[z], [self.z_dot]])

        # Compute the state feedback controller
        force_tilde = -self.K*x - self.ki*self.integrator

        # compute total torque
        force = self.saturate(force_tilde)
        return [force.item(0)]

    def differentiateZ(self, z):
        '''
            differentiate z
        '''
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

