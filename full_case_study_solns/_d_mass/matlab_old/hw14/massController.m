classdef massController < handle
    %----------------------------
    properties
        x_hat
        d_hat
        force_d1
        integrator
        error_d1
        K
        ki
        L
        Ld
        A
        B
        C
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %----------------------------
        function self = massController(P)
            % initialized object properties
            self.x_hat = [0.0; 0.0];
            self.d_hat = 0.0;
            self.force_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L;
            self.Ld = P.Ld;
            self.A  = P.A;
            self.B  = P.B;
            self.C  = P.C;
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        %----------------------------
        function force = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            
            % update the observer and extract z_hat
            self.updateObserver(y);
            z_hat = self.x_hat(1);

            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);

            % Compute the state feedback controller
            force_tilde = -self.K*self.x_hat - self.ki*self.integrator - self.d_hat;

            % compute total torque
            force = self.saturate(force_tilde);
            self.updateForce(force);  % the matlab handle class requires this to be in a method
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*(self.x_hat)...
                    + self.B*(self.force_d1+self.d_hat)...
                    + self.L*(y_m-self.C*self.x_hat)...
                    );
                self.d_hat = self.d_hat + self.Ts/N*(...
                    self.Ld*(y_m - self.C*self.x_hat)...
                    );
            end
        end
        %----------------------------
        function self = updateForce(self, force)
            self.force_d1 = force;
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