classdef massController
    % 
    %    This class inherits other controllers in order to organize multiple controllers.
    %
    %----------------------------
    properties
        zCtrl
        limit
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            self.zCtrl = PIDControl(P.kp, P.ki, P.kd, P.F_max, P.beta, P.Ts);
            % plant parameters known to controller
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            force_tilde = self.zCtrl.PID(z_r, z, false);
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