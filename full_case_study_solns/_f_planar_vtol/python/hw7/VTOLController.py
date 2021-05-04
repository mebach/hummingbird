import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import VTOLParamHW7 as P7

class VTOLController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        self.kp_h = P7.kp_h
        self.kd_h = P7.kd_h
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
        tau = 0.0
        return np.array([[F], [tau]])







