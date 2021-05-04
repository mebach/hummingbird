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
            self.zCtrl = PDControl(P.kp, P.kd, P.F_max, P.beta, P.Ts);
            % plant parameters known to controller
            self.limit = P.F_max;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            % compute the linearized torque using PID
            force_tilde = self.zCtrl.PD(z_r, z, false);
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