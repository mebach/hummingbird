import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import VTOLParamHW8 as P8
from PDControl import PDControl

class VTOLController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        self.zCtrl = PDControl(P8.kp_z, P8.kd_z, P.fmax, P.beta, P.Ts)
        self.hCtrl = PDControl(P8.kp_h, P8.kd_h, P.fmax, P.beta, P.Ts)
        self.thetaCtrl = PDControl(P8.kp_th, P8.kd_th, P.fmax, P.beta, P.Ts)

    def u(self, r, y):
        z_r = float(r[0])
        h_r = float(r[1])
        z = y[0]
        h = y[1]
        theta = y[2]
        F_tilde = self.hCtrl.PD(h_r, h, flag=False)
        F = F_tilde + P.Fe
        theta_ref = self.zCtrl.PD(z_r, z, flag=False)
        tau = self.thetaCtrl.PD(theta_ref, theta, flag=False)
        return np.matrix([[F], [tau]])







