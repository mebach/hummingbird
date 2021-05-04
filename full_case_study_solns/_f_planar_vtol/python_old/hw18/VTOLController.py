import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import loopshape_VTOL_lon as L_lon
import loopshape_VTOL_lat_in as L_lat_in
import loopshape_VTOL_lat_out as L_lat_out
import numpy as np

class VTOLController:
    def __init__(self):
        self.x_lon_C = np.zeros((L_lon.C_ss.A.shape[0], 1))
        self.x_lon_F = np.zeros((L_lon.F_ss.A.shape[0], 1))
        self.x_lat_in_C  = np.zeros((L_lat_in.C_ss.A.shape[0], 1))
        self.x_lat_out_C = np.zeros((L_lat_out.C_ss.A.shape[0], 1))
        self.x_lat_F = np.zeros((L_lat_out.F_ss.A.shape[0], 1))
        self.limit = P.fmax  # Maximum force
        self.N = 10  #number of Euler integration steps for each sample

    def u(self, r, y):
        z_r = float(r[0])
        h_r = float(r[1])
        z = y[0]
        h = y[1]
        theta = y[2]

        # -----Longitudinal Control ------
        # prefilter
        self.updatePrefilterStateLon(h_r)
        h_r_filtered = L_lon.F_ss.C * self.x_lon_F + L_lon.F_ss.D * h_r
        # error signal for longitudinal loop
        error_lon = h_r_filtered - h
        # longitudinal controller
        self.updateControlStateLon(error_lon)
        F_tilde = L_lon.C_ss.C * self.x_lon_C + L_lon.C_ss.D * error_lon
        # total force
        F = P.Fe + F_tilde

        # -----Lateral Control ------
        # +++++Lateral Outer Loop +++++
        self.updatePrefilterStateLat(z_r)
        z_r_filtered = L_lat_out.F_ss.C * self.x_lat_F + L_lat_out.F_ss.D * z_r
        # error signal for outer loop
        error_lat_out = z_r_filtered - z
        # Outer loop control
        self.updateControlStateLatOut(error_lat_out)
        theta_r = L_lat_out.C_ss.C * self.x_lat_out_C + L_lat_out.C_ss.D * error_lat_out

        # +++++Lateral Inner Loop +++++
        # error signal for inner loop
        error_lat_in = theta_r - theta
        # Inner loop control
        self.updateControlStateLatIn(error_lat_in)
        tau = L_lat_in.C_ss.C * self.x_lat_in_C + L_lat_in.C_ss.D * error_lat_in

        u = np.concatenate((F, tau), axis=0)
        return u

    def updatePrefilterStateLon(self, h_r):
        for i in range(0, self.N):
            self.x_lon_F = self.x_lon_F + (P.Ts/self.N)*(
                L_lon.F_ss.A*self.x_lon_F + L_lon.F_ss.B*h_r
            )

    def updateControlStateLon(self, error_lon):
        for i in range(0, self.N):
            self.x_lon_C = self.x_lon_C + (P.Ts/self.N)*(
                L_lon.C_ss.A*self.x_lon_C + L_lon.C_ss.B*error_lon
            )

    def updatePrefilterStateLat(self, z_r):
        for i in range(0, self.N):
            self.x_lat_F = self.x_lat_F + (P.Ts/self.N)*(
                L_lat_out.F_ss.A*self.x_lat_F + L_lat_out.F_ss.B*z_r
            )

    def updateControlStateLatOut(self, error_lat_out):
        for i in range(0, self.N):
            self.x_lat_out_C = self.x_lat_out_C + (P.Ts/self.N)*(
                L_lat_out.C_ss.A*self.x_lat_out_C + L_lat_out.C_ss.B*error_lat_out
            )
    def updateControlStateLatIn(self, error_lat_in):
        for i in range(0, self.N):
            self.x_lat_in_C = self.x_lat_in_C + (P.Ts/self.N)*(
                L_lat_in.C_ss.A*self.x_lat_in_C + L_lat_in.C_ss.B*error_lat_in
            )

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

