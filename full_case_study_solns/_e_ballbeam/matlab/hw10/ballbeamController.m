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
            self.zCtrl = PIDControl(P.kp_z, P.ki_z, P.kd_z, P.theta_max, P.beta, P.Ts);
            self.thetaCtrl = PIDControl(P.kp_th, 0.0, P.kd_th, P.Fmax, P.beta, P.Ts);
        end
        %----------------------------
        function F = update(self, z_r, y)
            z = y(1);
            theta = y(2);
            % the reference angle for theta comes from the outer loop PD control
            theta_r = self.zCtrl.PID(z_r, z, false);
            % the force applied to the cart comes from the inner loop PD control
            F_tilde = self.thetaCtrl.PID(theta_r, theta, false);
            % equilibrium force
            Fe = self.m1*self.g*(z/self.length) + self.m2*self.g/2;
            % total force
            F = F_tilde + Fe;
        end
    end
end