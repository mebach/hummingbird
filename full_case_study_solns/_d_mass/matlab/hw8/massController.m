classdef massController
    properties
        kp
        kd
        limit
        Ts
    end
    methods
        %----------------------------
        function self = massController(P)
            self.kp = P.kp;
            self.kd = P.kd;
            self.limit = P.F_max;
            self.Ts = P.Ts
        end
        %----------------------------
        function force = update(self, z_r, state)
            z = state(1);
            zdot = state(2);
            force = self.kp * (z_r - z) - self.kd * zdot;
            % saturate the final output being sent to the dynamics.
            force = self.saturate(force);
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