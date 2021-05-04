classdef ballbeamController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp_z
        kd_z
        kp_th
        kd_th
        m1
        m2
        g
        length
        Fmax
    end
    %----------------------------
    methods
        %----------------------------
        function self = ballbeamController(P)
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            self.kp_z = P.kp_z;
            self.kd_z = P.kd_z;
            self.kp_th = P.kp_th;
            self.kd_th = P.kd_th;
            self.Fmax = P.Fmax;
        end
        %----------------------------
        function F = update(self, z_r, state)
            z = state(1);
            theta = state(2);
            zdot = state(3);
            thetadot = state(4);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.kp_z * (z_r - z) - self.kd_z * zdot;
            % the force applied to the cart comes from the inner loop PD control
            F_tilde = self.kp_th * (theta_r - theta) - self.kd_th * thetadot;
            % equilibrium force
            Fe = self.m1*self.g*(z/self.length) + self.m2*self.g/2;
            % total force
            F = F_tilde + Fe;
            % saturate the total force
            F = self.saturate(F, self.Fmax);
        end
        %----------------------------
        function out = saturate(self,u, limit)
            if abs(u) > limit
                u = limit*sign(u);
            end
            out = u;
        end

    end
end