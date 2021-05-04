import numpy as np
import ballbeamParam as P
import ballbeamParamHW11 as P11

class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.z_dot = 0.0             # derivative of z
        self.theta_dot = 0.0         # derivative of theta
        self.z_d1 = 0.0              # Position z delayed by 1 sample
        self.theta_d1 = 0.0          # Angle theta delayed by 1 sample
        self.K = P11.K                 # state feedback gain
        self.kr = P11.kr               # Input gain
        self.limit = P.Fmax          # Maximum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]
        theta = y[1]

        # differentiate z and theta
        self.differentiateZ(z)
        self.differentiateTheta(theta)

        # Construct the state
        x_tilde = np.matrix([[theta], [z-P.ze], [self.theta_dot], [self.z_dot]])
        zr_tilde = z_r - P.ze

        # Compute the state feedback controller
        F_tilde = -self.K*x_tilde + self.kr*zr_tilde

        F_unsat = P.Fe + F_tilde
        F = self.saturate(F_unsat)
        return [F.item(0)]

    def differentiateZ(self, z):
        '''
            differentiate z
        '''
        self.z_dot = self.beta*self.z_dot + (1-self.beta)*((z - self.z_d1) / self.Ts)
        self.z_d1 = z

    def differentiateTheta(self, theta):
        '''
            differentiate theta
        '''
        self.theta_dot = self.beta*self.theta_dot + (1-self.beta)*((theta - self.theta_d1) / self.Ts)
        self.theta_d1 = theta

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

