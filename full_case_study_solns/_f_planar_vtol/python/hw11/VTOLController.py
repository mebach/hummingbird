import numpy as np
import VTOLParam as P
import VTOLParamHW11 as P11

class VTOLController:
    def __init__(self):
        self.limit = P.fmax

    def update(self, r, x):
        z_r = r.item(0)
        h_r = r.item(1)
        z = x.item(0)
        h = x.item(1)
        theta = x.item(2)
        # Construct the states
        x_lon = np.array([[x.item(1)], [x.item(4)]])
        x_lat = np.array([[x.item(0)], [x.item(2)], [x.item(3)], [x.item(5)]])
        # Compute the state feedback controllers
        F_tilde = -P11.K_lon @ x_lon + P11.kr_lon*h_r
        F = P.Fe/np.cos(theta) + F_tilde.item(0)
        tau = -P11.K_lat @ x_lat + P11.kr_lat*z_r
        return np.array([[F], [tau.item(0)]])

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

