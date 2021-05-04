import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import VTOLParamHW7 as P7
from PDControl import PDControl

class VTOLController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.hCtrl = PDControl(P7.kp_h, P7.kd_h, P.fmax, P.beta, P.Ts)

    def u(self, r, y):
        z_r = float(r[0])
        h_r = float(r[1])
        z = y[0]
        h = y[1]
        theta = y[2]
        F_tilde = self.hCtrl.PD(h_r, h, flag=False)
        F = F_tilde + P.Fe
        tau = 0.0
        return np.matrix([[F], [tau]])







