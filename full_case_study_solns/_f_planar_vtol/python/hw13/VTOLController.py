import numpy as np
import VTOLParam as P
import VTOLParamHW13 as P13

class VTOLController:
    def __init__(self):
        self.xhat_lon = np.array([[0.0], [0.0]])
        self.xhat_lat = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.integrator_z = 0.0      # integrator on position z
        self.error_z_d1 = 0.0        # error signal delayed by 1 sample
        self.integrator_h = 0.0      # integrator on altitude h
        self.error_h_d1 = 0.0        # error signal delayed by 1 sample
        self.F_d1 = 0.0  # Force signal delayed by 1 sample
        self.tau_d1 = 0.0  # torque signal delayed by 1 sample
        self.limit = P.fmax
        self.Ts = P.Ts

    def update(self, r, y):
        z_r = r.item(0)
        h_r = r.item(1)
        y_lat = np.array([[y.item(0)],[y.item(2)]])
        y_lon = y.item(1)
        # update the observers
        xhat_lat = self.update_lat_observer(y_lat)
        xhat_lon = self.update_lon_observer(y_lon)
        z_hat = xhat_lat.item(0)
        h_hat = xhat_lon.item(0)
        theta_hat = xhat_lat.item(1)

        # integrate error
        error_z = z_r - z_hat
        self.integrateErrorZ(error_z)
        error_h = h_r - h_hat
        self.integrateErrorH(error_h)

        # Construct the states
        # Compute the state feedback controllers
        F_tilde = -P13.K_lon @ xhat_lon \
                  - P13.ki_lon * self.integrator_h
        F = P.Fe/np.cos(theta_hat) + F_tilde.item(0)
        tau = -P13.K_lat @ xhat_lat \
              - P13.ki_lat*self.integrator_z
        u = np.array([[F], [tau.item(0)]])
        self.F_d1 = F
        self.tau_d1 = tau.item(0)
        return u, xhat_lat, xhat_lon

    def update_lat_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lat(self.xhat_lat, y_m)
        F2 = self.observer_f_lat(self.xhat_lat + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lat(self.xhat_lat + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lat(self.xhat_lat + self.Ts * F3, y_m)
        self.xhat_lat += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.xhat_lat

    def observer_f_lat(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = P13.A_lat @ x_hat \
                   + P13.B_lat * self.tau_d1 \
                   + P13.L_lat @ (y_m - P13.C_lat @ x_hat)
        return xhat_dot

    def update_lon_observer(self, y_m):
        # update the observer using RK4 integration
        F1 = self.observer_f_lon(self.xhat_lon, y_m)
        F2 = self.observer_f_lon(self.xhat_lon + self.Ts / 2 * F1, y_m)
        F3 = self.observer_f_lon(self.xhat_lon + self.Ts / 2 * F2, y_m)
        F4 = self.observer_f_lon(self.xhat_lon + self.Ts * F3, y_m)
        self.xhat_lon += self.Ts / 6 * (F1 + 2 * F2 + 2 * F3 + F4)
        return self.xhat_lon

    def observer_f_lon(self, x_hat, y_m):
        # xhatdot = A*xhat + B*u + L(y-C*xhat)
        xhat_dot = P13.A_lon @ x_hat \
                   + P13.B_lon * (self.F_d1 - P.Fe) \
                   + P13.L_lon @ (y_m - P13.C_lon @ x_hat)
        return xhat_dot

    def integrateErrorZ(self, error_z):
        self.integrator_z = self.integrator_z \
                            + (P.Ts/2.0)*(error_z + self.error_z_d1)
        self.error_z_d1 = error_z

    def integrateErrorH(self, error_h):
        self.integrator_h = self.integrator_h \
                            + (P.Ts/2.0)*(error_h + self.error_h_d1)
        self.error_h_d1 = error_h

    def saturate(self,u):
        if abs(u) > self.limit:
            u = self.limit*np.sign(u)
        return u

