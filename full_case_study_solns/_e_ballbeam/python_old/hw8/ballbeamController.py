import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import ballbeamParamHW8 as P8
from PDControl import PDControl

class ballbeamController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.zCtrl = PDControl(P8.kp_z, P8.kd_z, P8.theta_max, P.beta, P.Ts)
        self.thetaCtrl = PDControl(P8.kp_th, P8.kd_th, P.Fmax, P.beta, P.Ts)

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]
        theta = y[1]
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.zCtrl.PD(z_r, z, flag=False)
        # the force applied to the cart comes from the inner loop PD control
        F_tilde = self.thetaCtrl.PD(theta_r, theta, flag=False)
        # equilibrium force
        Fe = P.m1*P.g*(z/P.length) + P.m2*P.g/2.0
        # total force
        F = F_tilde + Fe
        return [F]







