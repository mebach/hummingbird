classdef ballbeamController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        thetaCtrl
        m1
        m2
        g
        length
    end
    %----------------------------
    methods
        %----------------------------
        function self = ballbeamController(P)
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            % Instantiates the PD control objects
            self.zCtrl = PDControl(P.kp_z, P.kd_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PDControl(P.kp_th, P.kd_th, P.Fmax, P.beta, P.Ts);
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.zCtrl.PD(z_r, z, false);
            % the force applied to the cart comes from the inner loop PD control
            F_tilde = self.thetaCtrl.PD(theta_r, theta, false);
            % equilibrium force
            Fe = self.m1*self.g*(z/self.length) + self.m2*self.g/2;
            % total force
            F = F_tilde + Fe;
            % saturate the total force
            F = self.thetaCtrl.saturate(F);
        end
    end
end