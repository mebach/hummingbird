classdef ballbeamController < handle
    %----------------------------
    properties
        m1
        m2
        g
        length
        z_dot
        theta_dot
        z_d1
        theta_d1
        K
        kr
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
            self.z_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.K = P.K;
            self.kr = P.kr;
            self.limit = P.Fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function F = u(self, y_r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = y_r;
            z = y(1);
            theta = y(2);

            % differentiate z and theta
            self.differentiateZ(z);
            self.differentiateTheta(theta);

            z_e = self.length/2;
            % NOTE:  remember the feedback control should actually be
            % u_tilde = -K*x_tilde + kr*zd_tilde
            % since the only value that deviates from zero is z, we have
            x_tilde = [theta; z-z_e; self.theta_dot; self.z_dot];
            % equilibrium force
            F_e = 0.5*self.m2*self.g + self.m1*self.g*z_e/self.length;
            % compute the state feedback controller
            zr_tilde = z_r - z_e;
            F_tilde = -self.K*x_tilde + self.kr*zr_tilde;
            
            % total force
            F_unsat = F_tilde + F_e;
            F = self.saturate(F_unsat);
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
        function out = saturate(self,u)
            if abs(u) > self.limit
                u = self.limit*sign(u);
            end
            out = u;
        end
    end
end