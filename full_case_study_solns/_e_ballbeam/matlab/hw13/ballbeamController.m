classdef ballbeamController < handle
    properties
        m1
        m2
        g
        length
        x_hat
        F_d1
        integrator
        error_d1
        K
        ki
        L
        A
        B
        C
        limit
        Ts
    end
    methods
        %--Constructor--------------------------
        function self = ballbeamController(P)
            self.m1 = P.m1;
            self.m2 = P.m2;
            self.g = P.g;
            self.length = P.length;
            % initialized object properties
            self.x_hat = [0.0; 0.0; 0.0; 0.0];
            self.F_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L;
            self.A  = P.A;
            self.B  = P.B;
            self.C  = P.C;
            self.limit = P.Fmax;
            self.Ts = P.Ts;
        end
        %----------------------------
        function [F, x_hat] = update(self, z_r, y)
            %x_hat = y;
            % update the observer
            x_hat = self.updateObserver(y);
            z_hat = x_hat(1);
            
            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);

            % construct the state
            xe = [self.length/2; 0; 0; 0];
            % feedback linearizing force
            F_fl = 0.5*self.m2*self.g + self.m1*self.g*z_hat/self.length;
            % compute the state feedback controller
            F_tilde = - self.K * (x_hat - xe)...
                - self.ki * self.integrator;    
            F = self.saturate(F_fl + F_tilde);
            self.F_d1 = F_tilde;
        end
        function x_hat = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.x_hat, y);
            F2 = self.observer_f(self.x_hat + self.Ts/2*F1, y);
            F3 = self.observer_f(self.x_hat + self.Ts/2*F2, y);
            F4 = self.observer_f(self.x_hat + self.Ts*F3, y);
            self.x_hat = self.x_hat + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            x_hat = self.x_hat;
        end
        function x_hat_dot = observer_f(self, x_hat, y)
            x_e = [self.length/2; 0; 0; 0];
            z_hat = x_hat(1);
            % feedback linearizing force
            F_fl = 0.5*self.m2*self.g + self.m1*self.g*z_hat/self.length;
            x_hat_dot = self.A * (x_hat - x_e)...
                    + self.B * self.F_d1...
                    + self.L * (y - self.C * x_hat);
        end
        function self = integrateError(self, error)
            self.integrator = self.integrator...
                + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end