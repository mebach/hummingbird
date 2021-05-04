#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate

import numpy as np
#Import control library
import control as ctrl

class Controller(ControllerBase):
    def init_control(self, param):
        # Generally useful values
        b_theta = param.lT / (param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J1y + param.J2y) # page 26
        JT = param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J2z + param.m3 * (param.l3x**2 + param.l3y**2) # page 26
        Fe = (param.m1 * param.l1 + param.m2 * param.l2) * param.g / param.lT # page 26
        b_psi = (param.lT * Fe) / (JT + param.J1z) # page 34

        tr_theta = 1.4
        zeta_theta = 0.707
        omega_n_theta = 2.2 / tr_theta

        tr_phi = 0.3
        zeta_phi = 0.7
        omega_n_phi = 2.2 / tr_phi

        tr_psi = 10 * tr_phi
        zeta_psi = 0.7
        omega_n_psi = 2.2 / tr_psi

        # State space matrices for longitude control
        theta_c = ????
        x_lon = np.array([[state.pitch],
                          [state.pitch_dot]])
        A_lon = np.array([[0, 1],
                          [0, 0]])
        B_lon = np.array([[0],
                          [b_theta]])
        C_lon = np.array([[1, 0]])

        # Pole placements for longitude control, NOTE: This is a single row vector, the multiple rows is for visual aid
        P_lon = [complex(-zeta_theta*omega_n_theta, omega_n_theta*np.sqrt(1-zeta_theta**2)),
                 complex(-zeta_theta*omega_n_theta, -omega_n_theta*np.sqrt(1-zeta_theta**2))]

        # Check for controllability
        CC_lon = ctrl.ctrb(A_lon, B_lon)
        if np.linalg.matrix_rank(CC_lon) == np.size(A_lon,1):
            print(’the system is controllable’)
        else:
            print(’the system is not controllable’)

        # Find gains
        K_lon = ctrl.acker(A_lon, B_lon, P_lon)
        C_pitch = np.array([[1, 0]])
        kr_lon = -1/(C_pitch @ np.linalg.inv(A_lon - B_lon @ K_lon) @ B_lon)

        # State space matrices for latitude control
        psi_c = ????
        x_lat = np.array([[state.phi],
                          [state.psi],
                          [state.phi_dot],
                          [state.psi_dot]])
        A_lat = np.array([[0, 0, 1, 0],
                          [0, 0, 0, 1],
                          [0, 0, 0, 0],
                          [(param.lT*Fe)/(JT + param.J1z), 0, 0, 0]])
        B_lat = np.array([[0],
                          [0],
                          [1/param.J1x],
                          [0]])
        C_lat = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])

        # Pole placements for latitude control, NOTE: This is a single row vector, the multiple rows is for visual aid
        P_lat = [complex(-zeta_phi*omega_n_phi, omega_n_phi*np.sqrt(1-zeta_phi**2)),
                 complex(-zeta_phi*omega_n_phi, -omega_n_phi*np.sqrt(1-zeta_phi**2)),
                 complex(-zeta_psi*omega_n_psi, omega_n_psi*np.sqrt(1-zeta_psi**2)),
                 complex(-zeta_psi*omega_n_psi, -omega_n_psi*np.sqrt(1-zeta_psi**2))]

        # Check for controllability
        CC_lat = ctrl.ctrb(A_lat, B_lat)
        if np.linalg.matrix_rank(CC_lat) == np.size(A_lat,1):
            print(’the system is controllable’)
        else:
            print(’the system is not controllable’)

        # Find gains
        K_lat = ctrl.acker(A_lat, B_lat, P_lat)
        C_yaw = np.array([[0, 1, 0, 0]])
        kr_lat = -1/(C_yaw @ np.linalg.inv(A_lat - B_lat @ K_lat) @ B_lat)

        # Calculate tau
        tau_tilde = -K_lat @ x_lat + kr_lat*psi_c
        T = tau_tilde

        # Calculate force
        force_tilde = Fe - K_lon @ x_lon + kr_lon * theta_c
        F_l = (param.m1 * param.l1 + param.m2 * param.l2) * param.g * np.cos(state.pitch) / param.lT
        F = force_tilde + F_l

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
