classdef massController
    properties
        zCtrl
        limit
    end
    methods
        %----------------------------
        function self = massController(P)
            self.zCtrl = PIDControl(P.kp, P.ki, P.kd, P.F_max, P.beta, P.Ts);
            self.limit = P.F_max;
        end
        %----------------------------
        function force = update(self, z_r, y)
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