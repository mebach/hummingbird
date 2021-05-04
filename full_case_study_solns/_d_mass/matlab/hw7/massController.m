classdef massController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        kp
        kd
        limit
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            self.kp = P.kp;
            self.kd = P.kd;
            self.limit = P.F_max;
        end
        %----------------------------
        function force = update(self, z_r, state)
            z = state(1);
            zdot = state(2);
            
            % compute the linearized torque using PID
            force_tilde = self.kp * (z_r - z) - self.kd * zdot;
            % compute total torque
            force = self.saturate(force_tilde);
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