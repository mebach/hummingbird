import numpy as np
import ballbeamParam as P
import ballbeamParamHW14 as P14

class ballbeamController:
    # state feedback control using dirty derivatives to estimate zdot and thetadot
    def __init__(self):
        self.x_hat = np.matrix([
            [0.0],  # initial estimate for z_hat
            [0.0],  # initial estimate for theta_hat
            [0.0],  # initial estimate for z_hat_dot
            [0.0]])  # initial estimate for theta_hat_dot
        self.d_hat = 0.0             # estimate of the disturbance
        self.F_d1 = 0.0  # Computed Force, delayed by one sample
        self.integrator = 0.0        # integrator
        self.error_d1 = 0.0          # error signal delayed by 1 sample
        self.K = P14.K               # state feedback gain
        self.ki = P14.ki             # Integral gain
        self.L = P14.L               # observer gain
        self.Ld = P14.Ld               # gain for disturbance observer
        self.A = P14.A               # system model
        self.B = P14.B
        self.C = P14.C
        self.limit = P.Fmax          # Maximum force
        self.beta = P.beta           # dirty derivative gain
        self.Ts = P.Ts               # sample rate of controller

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]
        theta = y[1]

        # update the observer and extract z_hat
        self.updateObserver(y)
        z_hat = self.x_hat[1]

        # integrate error
        error = z_r - z
        self.integrateError(error)

        # Construct the state
        xe = np.matrix([[0.0], [P.ze], [0.0], [0.0]])
        x_tilde = self.x_hat - xe

        # Compute the state feedback controller
        F_tilde = -self.K*x_tilde - self.ki*self.integrator
        F_unsat = P.Fe + F_tilde - self.d_hat
        F = self.saturate(F_unsat)
        self.integratorAntiWindup(F, F_unsat)
        self.updateForce(F)
        return [F.item(0)]

    def updateObserver(self, y_m):
        N = 10
        y = np.matrix([
            [y_m[1]],
            [y_m[0]]])
        xe = np.matrix([[0.0], [P.ze], [0.0], [0.0]])
        for i in range(0, N):
            self.x_hat = self.x_hat + self.Ts/float(N)*(
                self.A*(self.x_hat - xe)
                + self.B*(self.F_d1 - P.Fe + self.d_hat)
                + self.L*(y-self.C*self.x_hat)
            )
            self.d_hat = self.d_hat + self.Ts/float(N)*(
                self.Ld*(y-self.C*self.x_hat)
            )

    def updateForce(self, F):
        self.F_d1 = F

    def integrateError(self, error):
        self.integrator = self.integrator + (self.Ts/2.0)*(error + self.error_d1)
        self.error_d1 = error

    def integratorAntiWindup(self, F, F_unsat):
        # integrator anti - windup
        if self.ki != 0.0:
            self.integrator = self.integrator + P.Ts/self.ki*(F-F_unsat)

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

