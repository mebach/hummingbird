import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import VTOLParamHW8 as P8

class VTOLController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        self.kp_h = P8.kp_h
        self.kd_h = P8.kd_h
        self.kp_z = P8.kp_z
        self.kd_z = P8.kd_z
        self.kp_th = P8.kp_th
        self.kd_th = P8.kd_th
        self.fmax = P.fmax

    def update(self, reference, state):
        z_r = reference.item(0)
        h_r = reference.item(1)
        z = state.item(0)
        h = state.item(1)
        theta = state.item(2)
        zdot = state.item(3)
        hdot = state.item(4)
        thetadot = state.item(5)
        F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot
        F = F_tilde + P.Fe
        theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot
        tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot
        return np.array([[F], [tau]])







