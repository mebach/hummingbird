import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import ballbeamParamHW10 as P10
from PIDControl import PIDControl

class ballbeamController:
    ''' 
        This class inherits other controllers in order to organize multiple controllers.
    '''

    def __init__(self):
        # Instantiates the SS_ctrl object
        self.zCtrl = PIDControl(P10.kp_z, P10.ki_z, P10.kd_z, P10.theta_max, P.beta, P.Ts)
        self.thetaCtrl = PIDControl(P10.kp_th, 0.0, P10.kd_th, P.Fmax, P.beta, P.Ts)

    def u(self, y_r, y):
        # y_r is the referenced input
        # y is the current state
        z_r = y_r[0]
        z = y[0]
        theta = y[1]
        # the reference angle for theta comes from the outer loop PD control
        theta_r = self.zCtrl.PID(z_r, z, flag=False)
        # the force applied to the cart comes from the inner loop PD control
        F_tilde = self.thetaCtrl.PID(theta_r, theta, flag=False)
        # equilibrium force
        Fe = P.m1*P.g*(z/P.length) + P.m2*P.g/2.0
        # total force
        F = F_tilde + Fe
        return [F]







