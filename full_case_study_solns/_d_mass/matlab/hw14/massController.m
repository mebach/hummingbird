classdef massController < handle
    %----------------------------
    properties
        observer_state
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
        Ts
    end
    methods
        %----------------------------
        function self = massController(P)
            % initialized object properties
            self.observer_state = [0.0; 0.0; 0.0];
            self.force_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L2;
            self.A  = P.A2;
            self.B  = P.B1;
            self.C  = P.C2;
            self.limit = P.F_max;
            self.Ts = P.Ts;
        end
        function [force, x_hat] = update(self, z_r, y)
            % update the observer and extract z_hat
            [x_hat, d_hat] = self.updateObserver(y);
            z_hat = x_hat(1);

            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);

            % Compute the state feedback controller
            force_tilde = -self.K * x_hat...
                - self.ki * self.integrator...
                - d_hat;

            % compute total torque
            force = self.saturate(force_tilde);
            self.force_d1 = force;
        end
        function [x_hat, d_hat] = updateObserver(self, y)
            % update observer using RK4 integration
            F1 = self.observer_f(self.observer_state, y);
            F2 = self.observer_f(self.observer_state + self.Ts/2*F1, y);
            F3 = self.observer_f(self.observer_state + self.Ts/2*F2, y);
            F4 = self.observer_f(self.observer_state + self.Ts*F3, y);
            self.observer_state = self.observer_state...
                + self.Ts/6 * (F1 + 2*F2 + 2*F3 + F4);
            x_hat = self.observer_state(1:2);
            d_hat = self.observer_state(3);
        end
        function x_hat_dot = observer_f(self, observer_state, y)
            x_hat_dot = self.A * observer_state...
                    + self.B * self.force_d1...
                    + self.L * (y - self.C * observer_state);
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