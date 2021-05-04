classdef VTOLController
    %----------------------------
    properties
        hCtrl
        Fe
        kp_h
        kd_h
        fmax
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            self.kp_h = P.kp_h;
            self.kd_h = P.kd_h;
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
            % the force applied to the cart comes from the inner loop PD control
            F_tilde = self.kp_h * (h_r - h) - self.kd_h * hdot;
            % total force
            F = F_tilde + self.Fe;
            tau = 0;
            out = [F; tau];
        end
    end
end