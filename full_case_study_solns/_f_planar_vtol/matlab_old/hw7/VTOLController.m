classdef VTOLController
    %----------------------------
    properties
        hCtrl
        Fe
    end
    %----------------------------
    methods
        %----------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            % Instantiates the PD control objects
            self.hCtrl = PDControl(P.kp_h, P.kd_h, P.fmax, P.beta, P.Ts);
        end
        %----------------------------
        function out = u(self, r, y)
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);
            % the force applied to the cart comes from the inner loop PD control
            F_tilde = self.hCtrl.PD(h_r, h, false);
            % total force
            F = F_tilde + self.Fe;
            tau = 0;
            out = [F; tau];
        end
    end
end