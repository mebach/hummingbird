classdef VTOLController
    %----------------------------
    properties
        zCtrl
        hCtrl
        thetaCtrl
        Fe
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            % Instantiates the PD control objects
            self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.fmax, P.beta, P.Ts);
            self.hCtrl = PIDControl(P.kp_h, P.ki_h, P.kd_h, P.fmax, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.fmax, P.beta, P.Ts);
        end
        %----------------------------
        function out = u(self, r, y)
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);

            F_tilde = self.hCtrl.PID(h_r, h, false);
            theta_ref = self.zCtrl.PID(z_r, z, false);
            tau = self.thetaCtrl.PID(theta_ref, theta, false);
            
            % total force
            F = F_tilde + self.Fe;
            out = [F; tau];
        end
    end
end