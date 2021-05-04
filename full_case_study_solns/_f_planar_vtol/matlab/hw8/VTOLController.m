classdef VTOLController
    %----------------------------
    properties
        kp_z
        kd_z
        fmax
        kp_h
        kd_h
        kp_th
        kd_th
        mixing
        Fe
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            self.mixing = P.mixing;
            self.kp_z = P.kp_z;
            self.kd_z = P.kd_z;
            self.kp_h = P.kp_h;
            self.kd_h = P.kd_h;
            self.kp_th = P.kp_th;
            self.kd_th = P.kd_th;
            self.fmax = P.fmax;
        end
        %----------------------------
        function out = update(self, reference, state)
            z_r = reference(1);
            h_r = reference(2);
            z = state(1);
            h = state(2);
            theta = state(3);
            zdot = state(4);
            hdot = state(5);
            thetadot = state(6);

            F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot;
            theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot;
            tau = self.kp_th * (theta_r - theta) - self.kd_th * thetadot;
            
            % total force
            F = F_tilde + self.Fe;
            
            % need to saturate the individual motors.  Convert back to f_r
            % and f_l to saturate each command individually, then turn back
            % to the saturated tau, F values.
            out = [F; tau];
            u = self.mixing*out;
            u(1) = self.saturate(u(1), self.fmax);
            u(2) = self.saturate(u(2), self.fmax);
            out = inv(self.mixing)*u;
                        
        end
        %----------------------------
        function out = saturate(self, u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end
    end
end