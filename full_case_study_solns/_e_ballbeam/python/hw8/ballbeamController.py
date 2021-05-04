import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import ballbeamParamHW8 as P8
from PDControl import PDControl

class ballbeamController:
    def __init__(self):
        self.kp_z = P8.kp_z
        self.kd_z = P8.kd_z
        self.kp_th = P8.kp_th
        self.kd_th = P8.kd_th
        self.theta_max = P8.theta_max

    def update(self, z_r, state):
        z = state.item(0)
        theta = state.item(1)
        zdot = state.item(2)
        thetadot = state.item(3)
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        # the force applied to the cart comes from the inner loop PD control
        F_tilde = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        # feedback linearizing force
        F_fl = P.m1*P.g*(z/P.length) + P.m2*P.g/2.0
        # total force
        F = F_tilde + F_fl
        return F







