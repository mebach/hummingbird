classdef massController < handle
    %----------------------------
    properties
        z_dot
        z_d1
        integrator
        error_d1
        K
        ki
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            % initialized object properties
            self.z_dot = 0.0;
            self.z_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.F_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            % differentiate z
            self.differentiateZ(z);

            % integrate error
            error = z_r - z;
            self.integrateError(error);

            % Construct the state
            x = [z; self.z_dot];
            
            % Compute the state feedback controller
            force_tilde = -self.K*x - self.ki*self.integrator;

            % compute total force
            force = self.saturate(force_tilde);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z-self.z_d1) / self.Ts);
            self.z_d1 = z;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end