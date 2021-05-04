classdef ballbeamController < handle
    %----------------------------
    properties
        m1
        m2
        g
        length
        x_hat
        d_hat
        F_d1
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
    %----------------------------
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
            self.d_hat = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.L  = P.L;
            self.Ld = P.Ld;
            self.A  = P.A;
            self.B  = P.B;
            self.C  = P.C;
            self.limit = P.Fmax;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);

            % update the observer and extract z_hat
            self.updateObserver([theta; z]);
            z_hat = self.x_hat(2);
            
            % integrate error
            error = z_r - z_hat;
            self.integrateError(error);

            % construct the state
            ze = self.length/2;
            xe = [0; ze; 0; 0];
            % equilibrium force
            Fe = 0.5*self.m2*self.g + self.m1*self.g*ze/self.length;
            % compute the state feedback controller
            F_unsat = Fe - self.K*(self.x_hat-xe)...
                - self.ki*self.integrator - self.d_hat;    
            F = self.saturate(F_unsat);
    
            self.integratorAntiWindup(F, F_unsat);
            self.updateForce(F);
        end
        %----------------------------
        function self = updateObserver(self, y_m)
            x_e = [0; self.length/2; 0; 0];
            F_e = 0.5*self.m2*self.g + 0.5*self.m1*self.g;
            N = 10;
            for i=1:N
                self.x_hat = self.x_hat + self.Ts/N*(...
                    self.A*(self.x_hat-x_e)...
                    + self.B*(self.F_d1-F_e+self.d_hat)...
                    + self.L*(y_m-self.C*self.x_hat));
                self.d_hat = self.d_hat + self.Ts/N*(...
                    self.Ld*(y_m - self.C*self.x_hat)...
                    );
            end
        end
        %----------------------------
        function self = updateForce(self, F)
            self.F_d1 = F;
        end
        %----------------------------
        function self = integrateError(self, error)
            self.integrator = self.integrator + (self.Ts/2.0)*(error+self.error_d1);
            self.error_d1 = error;
        end
        %----------------------------
        function self = integratorAntiWindup(self, F, F_unsat)
            % integrator anti-windup
            if self.ki~=0
                self.integrator = self.integrator + self.Ts/self.ki*(F-F_unsat);
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