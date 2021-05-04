classdef massController < handle
    %----------------------------
    properties
        z_dot
        z_d1
        K
        kr
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
            self.K = P.K;
            self.kr = P.kr;
            self.limit = P.F_max;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function force = update(self, z_r, y)
            z = y(1);
            
            % differentiate z
            self.differentiateZ(z);

            % Construct the state
            x = [z; self.z_dot];
            
            % Compute the state feedback controller
            force_tilde = -self.K*x + self.kr*z_r;

            % compute total torque
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
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end        
    end
end