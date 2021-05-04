classdef ballbeamController < handle
    %----------------------------
    properties
        m1
        m2
        g
        length
        init_flag
        z_dot
        theta_dot
        z_d1
        theta_d1
        integrator
        error_d1
        K
        ki
        limit
        beta
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
            self.init_flag = 1;
            self.z_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.integrator = 0.0;
            self.error_d1 = 0.0;
            self.K = P.K;
            self.ki = P.ki;
            self.limit = P.Fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = update(self, z_r, y)
            z = y(1);
            theta = y(2);

            % differentiate z and theta
            if self.init_flag ==1
                self.z_d1 = z;
                self.theta_d1 = theta;
                self.init_flag = 0;
            end
            self.differentiateZ(z);
            self.differentiateTheta(theta);
            
            % integrate error
            error = z_r - z;
            self.integrateError(error);

            % construct the state
            ze = self.length/2;
            xe = [ze; 0; 0; 0];
            x = [z; theta; self.z_dot; self.theta_dot];
            % feedback linearizing force
            F_fl = 0.5*self.m2*self.g + self.m1*self.g*z/self.length;
            % compute the state feedback controller
            F_unsat = F_fl - self.K*(x-xe) - self.ki*self.integrator;    
            F = self.saturate(F_unsat);
    
            self.integratorAntiWindup(F, F_unsat);
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        %----------------------------
        function self = differentiateTheta(self, theta)
            self.theta_dot = ...
                self.beta*self.theta_dot...
                + (1-self.beta)*((theta-self.theta_d1) / self.Ts);
            self.theta_d1 = theta;
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