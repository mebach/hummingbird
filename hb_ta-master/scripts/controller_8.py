#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate

import numpy as np

class Controller(ControllerBase):
    def init_control(self, param):
        # Generally useful values
        b_theta = param.lT / (param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J1y + param.J2y) # page 26
        JT = param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J2z + param.m3 * (param.l3x**2 + param.l3y**2) # page 26
        Fe = (param.m1 * param.l1 + param.m2 * param.l2) * param.g / param.lT # page 26
        b_psi = (param.lT * Fe) / (JT + param.J1z) # page 34

        # Longitudinal control parameters
        tr_theta = 1.5
        zeta_theta = 0.707
        omega_n_theta = 2.2 / tr_theta
        self.kp_theta = omega_n_theta**2 / b_theta
        self.kd_theta = 2 * zeta_theta * omega_n_theta / b_theta

        # Lateral control parameters
        tr_phi = 0.3
        zeta_phi = 0.7
        omega_n_phi = 2.2 / tr_phi
        self.kp_phi = omega_n_phi ** 2 * param.J1x
        self.kd_phi = 2 * zeta_phi * omega_n_phi * param.J1x

        tr_psi = 10 * tr_phi
        zeta_psi = 0.7
        omega_n_psi = 2.2 / tr_psi
        self.kp_psi = omega_n_psi ** 2 / b_psi
        self.kd_psi = 2 * zeta_psi * omega_n_psi / b_psi

        print("Gains")
        print("-----")
        print("PITCH", "kp:", self.kp_theta, "kd:", self.kd_theta)
        print("YAW", "kp:", self.kp_psi, "kd:", self.kd_psi)
        print("ROLL", "kp:", self.kp_phi, "kd:", self.kd_phi)
        print("Fe", Fe)

        # Memory values
        self.theta_k1 = 0.0
        self.theta_d_k1 = 0.0
        self.psi_k1 = 0.0
        self.psi_d_k1 = 0.0
        self.phi_k1 = 0.0
        self.phi_d_k1 = 0.0

    def dirty_derivative(self, cur, prev, prev_dot, dt, sigma=0.05):
        return (2 * sigma - dt) / (2 * sigma + dt) * prev_dot + 2 / (2 * sigma + dt) * (cur - prev)

    def compute_control(self, param, state, reference, dt):
        left = 0.0
        right = 0.0

        # Calculate forces
        self.theta_d_k1 = self.dirty_derivative(state.pitch, self.theta_k1, self.theta_d_k1, dt)
        self.theta_k1 = state.pitch
        F_tilde_ = self.kp_theta * (reference.pitch - state.pitch) - self.kd_theta * self.theta_d_k1
        F_l_ = (param.m1 * param.l1 + param.m2 * param.l2) * param.g * np.cos(state.pitch) / param.lT
        F = F_tilde_ + F_l_

        # Calculate torque
        self.psi_d_k1 = self.dirty_derivative(state.yaw, self.psi_k1, self.psi_d_k1, dt)
        self.psi_k1 = state.yaw
        phi_c = self.kp_psi * (reference.yaw - state.yaw) - self.kd_psi * self.psi_d_k1

        self.phi_d_k1 = self.dirty_derivative(state.roll, self.phi_k1, self.phi_d_k1, dt)
        self.phi_k1 = state.roll
        T = self.kp_phi * (phi_c - state.roll) - self.kd_phi * self.phi_d_k1

        # Set and saturate commands
        left = 1 / (2.0 * param.km) * (F + T / param.d)
        right = 1 / (2.0 * param.km) * (F - T / param.d)

        if left > 0.7 or right > 0.7:
            print("Saturation!", left, right)

        left = saturate(left, 0, 0.7)
        right = saturate(right, 0, 0.7)        

        return Command(left=left, right=right)


if __name__ == '__main__':
    c = Controller()
    c.run()
