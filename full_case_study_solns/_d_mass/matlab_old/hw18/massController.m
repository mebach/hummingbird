classdef massController < handle
    %----------------------------
    properties
        x_C
        x_F
        A_F
        B_F
        C_F
        D_F
        A_C
        B_C
        C_C
        D_C
        limit
        beta
        Ts
        N
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            % initialized object properties
            self.x_C = zeros(size(P.A_C,1),1);
            self.x_F = zeros(size(P.A_F,1),1); 
            self.A_F = P.A_F;
            self.B_F = P.B_F;
            self.C_F = P.C_F;
            self.D_F = P.D_F;
            self.A_C = P.A_C;
            self.B_C = P.B_C;
            self.C_C = P.C_C;
            self.D_C = P.D_C;
            self.limit = P.F_max;
            self.Ts = P.Ts;
            self.N = 10; % number of Euler integration steps for each sample
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            
            % solve differential equation defining prefilter
            self.updatePrefilterState(z_r);
            z_r_filtered = self.C_F*self.x_F + self.D_F*z_r;

            % integrate error
            error = z_r_filtered - z;

            % Compute the control C(s)
            self.updateControlState(error);
            force_tilde = self.C_C*self.x_C + self.D_C*error;

            % compute total torque
            force = self.saturate(force_tilde);
        end
        %----------------------------
        function self = updatePrefilterState(self, z_r)
            for i=1:self.N
                self.x_F = self.x_F + self.Ts/self.N*(... 
                    self.A_F*self.x_F + self.B_F*z_r...
                    );
            end
        end
        %----------------------------
        function self = updateControlState(self, error)
            for i=1:self.N
                self.x_C = self.x_C + self.Ts/self.N*(...
                    self.A_C*self.x_C + self.B_C*error...
                    );
            end
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