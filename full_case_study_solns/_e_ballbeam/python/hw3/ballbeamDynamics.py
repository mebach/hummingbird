import numpy as np
import ballbeamParam as P

class ballbeamDynamics:
    def __init__(self, alpha=0.0):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # z initial ball position
            [P.theta0],  # Theta initial beam angle
            [P.zdot0],  # zdot initial ball velocity
            [P.thetadot0],  # Thetadot initial beam angular velocity
        ])
        self.Ts = P.Ts
        self.m1 = P.m1 * (1+2*alpha*np.random.rand()-alpha)
        self.m2 = P.m2 * (1+2*alpha*np.random.rand()-alpha)
        self.length = P.length * (1+2*alpha*np.random.rand()-alpha)
        self.g = P.g

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        # return xdot = f(x,u)
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        F = u
        # The equations of motion.
        zddot = (1.0/self.m1)*(self.m1*z*thetadot**2
                               - self.m1*self.g*np.sin(theta))

        thetaddot = (1.0/((self.m2*self.length**2)/3.0
                          + self.m1*z**2))*(-2.0*self.m1*z*zdot*thetadot
                                            - self.m1*self.g*z*np.cos(theta)
                    - self.m2*self.g*self.length/2.0*np.cos(theta)
                    + self.length*F*np.cos(theta))
        # build xdot and return
        xdot = np.array([[zdot], [thetadot], [zddot], [thetaddot]])
        return xdot

    def h(self):
        # return y = h(x)
        z = self.state.item(0)
        theta = self.state.item(1)
        y = np.array([[z], [theta]])
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)