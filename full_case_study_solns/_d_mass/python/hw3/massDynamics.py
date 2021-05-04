import numpy as np 
import random
import massParam as P


class massDynamics:
    def __init__(self):
        # Initial state conditions
        self.state = np.array([
            [P.z0],  # initial mass position
            [P.zdot0],  # initial mass velocity
        ])
        #################################################
        # The parameters for any physical system are never known exactly.  Feedback
        # systems need to be designed to be robust to this uncertainty.  In the simulation
        # we model uncertainty by changing the physical parameters by a uniform random variable
        # that represents alpha*100 % of the parameter, i.e., alpha = 0.2, means that the parameter
        # may change by up to 20%.  A different parameter value is chosen every time the simulation
        # is run.
        alpha = 0.2  # Uncertainty parameter
        self.m = P.m * (1+2*alpha*np.random.rand()-alpha)  # Mass, kg
        self.k = P.k * (1+2*alpha*np.random.rand()-alpha)  # spring constant, m
        self.b = P.b * (1+2*alpha*np.random.rand()-alpha)  # Damping coefficient, Ns
        self.Ts = P.Ts  # sample rate at which the dynamics are propagated

    def update(self, u):
        # This is the external method that takes the input u at time
        # t and returns the output y at time t.
        self.rk4_step(u)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y

    def f(self, state, u):
        #  Return xdot = f(x,u),
        z = state.item(0)
        zdot = state.item(1)
        force = u
        # The equations of motion.
        zddot = (force - self.b*zdot - self.k*z)/self.m
        # build xdot and return
        xdot = np.array([[zdot], [zddot]])
        return xdot

    def h(self):
        # return y = h(x)
        z = self.state.item(0)
        y = np.array([[z]])
        return y

    def rk4_step(self, u):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, u)
        F2 = self.f(self.state + self.Ts / 2 * F1, u)
        F3 = self.f(self.state + self.Ts / 2 * F2, u)
        F4 = self.f(self.state + self.Ts * F3, u)
        self.state += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)

