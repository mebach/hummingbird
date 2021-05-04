classdef VTOLController
    %----------------------------
    properties
        zCtrl
        hCtrl
        thetaCtrl
        mixing
        Fe
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            self.mixing = P.mixing;
            % Instantiates the PD control objects
            self.zCtrl = PDControl(P.kp_z, P.kd_z, P.fmax, P.beta, P.Ts);
            self.hCtrl = PDControl(P.kp_h, P.kd_h, P.fmax, P.beta, P.Ts);
            self.thetaCtrl = PDControl(P.kp_th, P.kd_th, P.fmax, P.beta, P.Ts);
        end
        %----------------------------
        function out = u(self, r, y)
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);

            F_tilde = self.hCtrl.PD(h_r, h, false);
            theta_ref = self.zCtrl.PD(z_r, z, false);
            tau = self.thetaCtrl.PD(theta_ref, theta, false);
            
            % total force
            F = F_tilde + self.Fe;
            
            % need to saturate the individual motors.  Convert back to f_r
            % and f_l to saturate each command individually, then turn back
            % to the saturated tau, F values.
            out = [F; tau];
            u = self.mixing*out;
            u(1) = self.thetaCtrl.saturate(u(1));
            u(2) = self.thetaCtrl.saturate(u(2));
            out = inv(self.mixing)*u;
                        
        end
    end
end