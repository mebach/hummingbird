#!/usr/bin/env python3

from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate

import numpy as np
#Import control library
import control as ctrl

class Controller(ControllerBase):
    def init_control(self, param):
        self.debug = False
        # Generally useful values
        self.b_theta = param.lT / (param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J1y + param.J2y) # page 26
        self.JT = param.m1 * param.l1**2 + param.m2 * param.l2**2 + param.J2z + param.m3 * (param.l3x**2 + param.l3y**2) # page 26
        self.Fe = (param.m1 * param.l1 + param.m2 * param.l2) * param.g / param.lT # page 26
        # b_psi = (param.lT * self.Fe) / (self.JT + param.J1z) # page 34

        tr_theta = 0.75
        zeta_theta = 0.7
        omega_n_theta = 2.2 / tr_theta

        tr_phi = 0.4
        zeta_phi = 0.7
        omega_n_phi = 2.2 / tr_phi

        tr_psi = 4 * tr_phi
        zeta_psi = 0.7
        omega_n_psi = 2.2 / tr_psi

        # State space matrices for longitude control
        self.A_lon = np.array([[0, 1],
                          [0, 0]])
        self.B_lon = np.array([[0],
                          [self.b_theta]])
        self.C_lon = np.array([[1, 0]])

        # Pole placements for longitude control, NOTE: This is a single row vector, the multiple rows is for visual aid
        # Can be found with convolving function
        self.P_lon = [complex(-zeta_theta*omega_n_theta, omega_n_theta*np.sqrt(1-zeta_theta**2)),
                 complex(-zeta_theta*omega_n_theta, -omega_n_theta*np.sqrt(1-zeta_theta**2))]

        # Check for controllability
        CC_lon = ctrl.ctrb(self.A_lon, self.B_lon)
        if np.linalg.matrix_rank(CC_lon) == np.size(self.A_lon,1):
            print("Lon system is controllable")
        else:
            print("Lon system is not controllable")


        # State space matrices for latitude control
        self.A_lat = np.array([[0, 0, 1, 0],
                          [0, 0, 0, 1],
                          [0, 0, 0, 0],
                          [(param.lT*self.Fe)/(self.JT + param.J1z), 0, 0, 0]])
        self.B_lat = np.array([[0],
                          [0],
                          [1/param.J1x],
                          [0]])
        self.C_lat = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0]])

        # Pole placements for latitude control, NOTE: This is a single row vector, the multiple rows is for visual aid
        # Can be found with convolving function
        self.P_lat = [complex(-zeta_phi*omega_n_phi, omega_n_phi*np.sqrt(1-zeta_phi**2)),
                 complex(-zeta_phi*omega_n_phi, -omega_n_phi*np.sqrt(1-zeta_phi**2)),
                 complex(-zeta_psi*omega_n_psi, omega_n_psi*np.sqrt(1-zeta_psi**2)),
                 complex(-zeta_psi*omega_n_psi, -omega_n_psi*np.sqrt(1-zeta_psi**2))]

        # Check for controllability
        CC_lat = ctrl.ctrb(self.A_lat, self.B_lat)
        if np.linalg.matrix_rank(CC_lat) == np.size(self.A_lat,1):
            print("Lat system is controllable")
        else:
            print("Lat system is not controllable")

        # Find gains using ackermans formula and block diagram math
        self.K_lon = ctrl.acker(self.A_lon, self.B_lon, self.P_lon)
        C_pitch = np.array([[1, 0]])
        self.kr_lon = -1/(C_pitch @ np.linalg.inv(self.A_lon - self.B_lon @ self.K_lon) @ self.B_lon)
        self.K_lat = ctrl.acker(self.A_lat, self.B_lat, self.P_lat)
        C_yaw = np.array([[0, 1, 0, 0]])
        self.kr_lat = -1/(C_yaw @ np.linalg.inv(self.A_lat - self.B_lat @ self.K_lat) @ self.B_lat)

        if self.debug == True:
            print("\n")
            print("K_lon:", self.K_lon)
            print("kr_lon:", self.kr_lon)
            print("K_lat:", self.K_lat)
            print("kr_lat:", self.kr_lat)
            print("\n")
            input("")


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

        if self.debug:
            print("commanded pitch: ", reference.pitch)
            print("commanded yaw: ", reference.yaw)
            print("actual pitch: ", state.pitch)
            print("actual yaw: ", state.yaw)
        psi_c = reference.yaw
        theta_c = reference.pitch

        # Calculate the dirty derivatives for state vector x_lon and x_lat
        self.theta_d_k1 = self.dirty_derivative(state.pitch, self.theta_k1, self.theta_d_k1, dt)
        self.theta_k1 = state.pitch
        self.psi_d_k1 = self.dirty_derivative(state.yaw, self.psi_k1, self.psi_d_k1, dt)
        self.psi_k1 = state.yaw
        self.phi_d_k1 = self.dirty_derivative(state.roll, self.phi_k1, self.phi_d_k1, dt)
        self.phi_k1 = state.roll
        x_lat = np.array([[state.roll],
                          [state.yaw],
                          [self.phi_d_k1],
                          [self.psi_d_k1]])
        x_lon = np.array([[state.pitch],
                          [self.theta_d_k1]])
        # Calculate tau from block diagram math
        tau_tilde = -self.K_lat @ x_lat + self.kr_lat*psi_c
        T = tau_tilde

        # Calculate force from block diagram math
        force_tilde = self.Fe - self.K_lon @ x_lon + self.kr_lon * theta_c
        F = force_tilde

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


# #!/usr/bin/env python3
#
# from hb_common import ControllerBase, Params, State, ReferenceState, Command, saturate
# import numpy as np
#
#
# class Controller(ControllerBase):
#
#     def init_control(self, param):
#         self.thetaDotOld = 0.0
#         self.thetaOld = 0.0
#
#         self.phiDotOld = 0.0
#         self.phiOld = 0.0
#
#         self.psiDotOld = 0.0
#         self.psiOld = 0.0
#
#
#
#     def compute_control(self, param, state, reference, dt):
#         left = 0.0
#         right = 0.0
#
#         #Rise time and kp/kd calculations for the pitch
#         sigma = 0.05
#         zeta = 0.707
#         tr = 1.5
#         kpMOD = .34
#         kdMOD = .58
#         wn = np.pi/(2*tr*np.sqrt(1 - zeta**2))
#         b = param.lT/(param.m1*param.l1**2 + param.m2*param.l2**2 + param.J1y + param.J2y)
#         kp = wn**2/(b*kpMOD)
#         kd = 2*wn*zeta/(b*kdMOD)
#
#         #Rise time and kp/kd calculations for the roll
#         sigma_phi = 0.05
#         zeta_phi = 0.7
#         tr_phi = .3
#         kpMOD_phi = .5
#         kdMOD_phi = 1.2
#         wn_phi = np.pi/(2*tr_phi*np.sqrt(1 - zeta_phi**2))
#         b_phi = 1/param.J1x
#         kp_phi = wn_phi**2*kpMOD_phi/(b_phi)
#         kd_phi = 2*wn_phi*zeta_phi*kdMOD_phi/(b_phi)
#
#         #Rise time and kp/kd calculations for the yaw
#         sigma_psi = 0.05
#         zeta_psi = 0.7
#         M = 10
#         tr_psi = tr_phi*M
#         kpMOD_psi = 12
#         kdMOD_psi = 5
#         wn_psi = np.pi/(2*tr_psi*np.sqrt(1 - zeta_psi**2))
#         Fe = (param.m1*param.l1 + param.m2*param.l2)*param.g/param.lT
#         JT = param.m1*param.l1**2 + param.m2*param.l2**2 + param.J2z + param.m3*(param.l3x**2 + param.l3y**2)
#         b_psi = param.lT*Fe/(JT + param.J1z)
#         kp_psi = wn_psi**2*kpMOD_psi/(b_psi)
#         kd_psi = 2*wn_psi*zeta_psi*kdMOD_psi/(b_psi)
#
#         #DIRTY DERIVATIVES
#         #Digtal PD control for the pitch
#         thetaDot = (2*sigma - dt)*(self.thetaDotOld)/(2*sigma + dt) + 2*(state.pitch - self.thetaOld)/(2*sigma + dt)
#
#         #Force calculations for the pitch
#         FTilde = kp*(reference.pitch - state.pitch) - kd*thetaDot
#         Fl = (param.m1*param.l1 + param.m2*param.l2)*param.g*np.cos(state.pitch)/param.lT
#         F = (FTilde + Fl)
#
#         #Digital PD control for the yaw
#         psiDot = (2*sigma_psi - dt)*(self.psiDotOld)/(2*sigma_psi + dt) + 2*(state.yaw - self.psiOld)/(2*sigma_psi + dt)
#
#         #ReferenceRoll calculations for the yaw
#         referenceRoll = kp_psi*(reference.yaw - state.yaw) - kd_psi*psiDot
#
#         #Digital PD control for the roll
#         phiDot = (2*sigma_phi - dt)*(self.phiDotOld)/(2*sigma_phi + dt) + 2*(state.roll - self.phiOld)/(2*sigma_phi + dt)
#
#         #Torque Calculations for the roll
#         Torque = kp_phi*(referenceRoll - state.roll) - kd_phi*phiDot
#
#
#         #Assignments to left and right motors
#         leftKm = 1.03
#         rightKm = .97
#         left = (F + Torque/param.d)*leftKm/(2*param.km)
#         right = (F - Torque/param.d)*rightKm/(2*param.km)
#
#         left = saturate(left, 0, 1.0)
#         right = saturate(right, 0, 1.0)
#
#         self.thetaOld = state.pitch
#         self.thetaDotOld = thetaDot
#         self.psiOld = state.yaw
#         self.psiDotOld = psiDot
#         self.phiOld = state.roll
#         self.phiDotOld = phiDot
#
#         return Command(left=left, right=right)
#
#
# if __name__ == '__main__':
#     c = Controller()
#     c.run()
#
#
# # Simulation
# # def compute_control(self, param, state, reference, dt):
# #     left = 0.0
# #     right = 0.0
# #
# #     #Rise time and kp/kd calculations for the pitch
# #     sigma = 0.05
# #     zeta = 0.707
# #     tr = 1.5
# #     kpMOD = .34
# #     kdMOD = .58
# #     wn = np.pi/(2*tr*np.sqrt(1 - zeta**2))
# #     b = param.lT/(param.m1*param.l1**2 + param.m2*param.l2**2 + param.J1y + param.J2y)
# #     kp = wn**2/(b*kpMOD)
# #     kd = 2*wn*zeta/(b*kdMOD)
# #
# #     #Rise time and kp/kd calculations for the roll
# #     sigma_phi = 0.05
# #     zeta_phi = 0.7
# #     tr_phi = .2
# #     kpMOD_phi = 1
# #     kdMOD_phi = 1
# #     wn_phi = np.pi/(2*tr_phi*np.sqrt(1 - zeta_phi**2))
# #     b_phi = 1/param.J1x
# #     kp_phi = wn_phi**2/(b_phi*kpMOD_phi)
# #     kd_phi = 2*wn_phi*zeta_phi/(b_phi*kdMOD_phi)
# #
# #     #Rise time and kp/kd calculations for the yaw
# #     sigma_psi = 0.05
# #     zeta_psi = 0.7
# #     M = 10
# #     tr_psi = tr_phi*M
# #     kpMOD_psi = 1
# #     kdMOD_psi = 1
# #     wn_psi = np.pi/(2*tr_psi*np.sqrt(1 - zeta_psi**2))
# #     Fe = (param.m1*param.l1 + param.m2*param.l2)*param.g/param.lT
# #     JT = param.m1*param.l1**2 + param.m2*param.l2**2 + param.J2z + param.m3*(param.l3x**2 + param.l3y**2)
# #     b_psi = param.lT*Fe/(JT + param.J1z)
# #     kp_psi = wn_psi**2/(b_psi*kpMOD_psi)
# #     kd_psi = 2*wn_psi*zeta_psi/(b_psi*kdMOD_psi)
# #
# #     #DIRTY DERIVATIVES
# #     #Digtal PD control for the pitch
# #     thetaDot = (2*sigma - dt)*(self.thetaDotOld)/(2*sigma + dt) + 2*(state.pitch - self.thetaOld)/(2*sigma + dt)
# #
# #     #Force calculations for the pitch
# #     FTilde = kp*(reference.pitch - state.pitch) - kd*thetaDot
# #     Fl = (param.m1*param.l1 + param.m2*param.l2)*param.g*np.cos(state.pitch)/param.lT
# #     F = (FTilde + Fl)
# #
# #     #Digital PD control for the yaw
# #     psiDot = (2*sigma_psi - dt)*(self.psiDotOld)/(2*sigma_psi + dt) + 2*(state.yaw - self.psiOld)/(2*sigma_psi + dt)
# #
# #     #ReferenceRoll calculations for the yaw
# #     referenceRoll = kp_psi*(reference.yaw - state.yaw) - kd_psi*psiDot
# #
# #     #Digital PD control for the roll
# #     phiDot = (2*sigma_phi - dt)*(self.phiDotOld)/(2*sigma_phi + dt) + 2*(state.roll - self.phiOld)/(2*sigma_phi + dt)
# #
# #     #Torque Calculations for the roll
# #     Torque = kp_phi*(referenceRoll - state.roll) - kd_phi*phiDot
# #
# #
# #     #Assignments to left and right motors
# #     leftKm = 1
# #     rightKm = 1
# #     left = (F + Torque/param.d)/(2*leftKm*param.km)
# #     right = (F - Torque/param.d)/(2*rightKm*param.km)
# #
# #     left = saturate(left, 0, 1.0)
# #     right = saturate(right, 0, 1.0)
# #
# #     self.thetaOld = state.pitch
# #     self.thetaDotOld = thetaDot
# #     self.psiOld = state.yaw
# #     self.psiDotOld = psiDot
# #     self.phiOld = state.roll
# #     self.phiDotOld = phiDot
# #
# #     return Command(left=left, right=right)
