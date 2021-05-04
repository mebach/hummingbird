#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate

import numpy as np

class Controller(ControllerBase):
    def init_control(self, param):
        # Generally useful values
        b_theta = param.lT / (param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J1y + param.J2y)

        # Longitudinal control parameters
        tr_theta = 1.5
        zeta_theta = 0.707
        
        omega_n_theta = 2.2 / tr_theta
        self.kp_theta = omega_n_theta**2 / b_theta
        self.kd_theta = 2 * zeta_theta * omega_n_theta / b_theta

        # Memory values
        self.theta_k1 = 0.0
        self.theta_d_k1 = 0.0

    def dirty_derivative(self, cur, prev, prev_dot, dt, sigma=0.005):
        return (2 * sigma - dt) / (2 * sigma + dt) * prev_dot + 2 / (2 * sigma + dt) * (cur - prev)

    def compute_control(self, param, state, reference, dt):
        # Calculate forces
        theta_d = self.dirty_derivative(state.pitch, self.theta_k1, self.theta_d_k1, dt)
        F_tilde = self.kp_theta * (reference.pitch - state.pitch) - self.kd_theta * theta_d
        F_l = (param.m1 * param.l1 + param.m2 * param.l2) * param.g * np.cos(state.pitch) / param.lT
        F = F_tilde + F_l

        # Set and saturate commands
        left = F / (2.0 * param.km)
        right = F / (2.0 * param.km)

        if left > 0.7 or right > 0.7:
            print("Saturation!", left, right)

        left = saturate(left, 0, 0.7)
        right = saturate(right, 0, 0.7)

        # Update memory values
        self.theta_k1 = state.pitch
        self.theta_d_k1 = theta_d

        return Command(left=left, right=right)


if __name__ == '__main__':
    c = Controller()
    c.run()
