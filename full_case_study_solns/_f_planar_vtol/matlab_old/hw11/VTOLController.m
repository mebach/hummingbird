classdef VTOLController < handle
    %----------------------------
    properties
        Fe
        z_dot
        h_dot
        theta_dot
        z_d1
        h_d1
        theta_d1
        K_lon
        kr_lon
        K_lat
        kr_lat
        limit
        beta
        Ts
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            % initialized object properties
            self.z_dot = 0.0;
            self.h_dot = 0.0;
            self.theta_dot = 0.0;
            self.z_d1 = 0.0;
            self.h_d1 = 0.0;
            self.theta_d1 = 0.0;
            self.K_lon = P.K_lon;
            self.kr_lon = P.kr_lon;
            self.K_lat = P.K_lat;
            self.kr_lat = P.kr_lat;
            self.limit = P.fmax;
            self.beta = P.beta;
            self.Ts = P.Ts;
        end
        %----------------------------
        function out = u(self, r, y)
            % y_r is the referenced input
            % y is the current state
            z_r = r(1);
            h_r = r(2);
            z = y(1);
            h = y(2);
            theta = y(3);

            % differentiate z and theta
            self.differentiateZ(z);
            self.differentiateH(h);
            self.differentiateTheta(theta);

            % NOTE:  remember the feedback control should actually be
            % u_tilde = -K*x_tilde + kr*zd_tilde
            % since the only value that deviates from zero is z, we have
            x_lon = [h; self.h_dot];
            % equilibrium force
            Fe_ = self.Fe/cos(theta); 
                % divide Fe by cos(theta) so that force is right during
                % lateral translations.
            % compute the state feedback controller
            F_tilde = -self.K_lon*x_lon + self.kr_lon*h_r;
            F = Fe_ + F_tilde;

            % lateral control for position
            % construct the state
            x_lat = [z; theta; self.z_dot; self.theta_dot];
            % compute the state feedback controller
            tau = -self.K_lat*x_lat + self.kr_lat*z_r;
            
            out = [F; tau];
        end
        %----------------------------
        function self = differentiateZ(self, z)
            self.z_dot = ...
                self.beta*self.z_dot...
                + (1-self.beta)*((z - self.z_d1) / self.Ts);
            self.z_d1 = z;            
        end
        %----------------------------
        function self = differentiateH(self, h)
            self.h_dot = ...
                self.beta*self.h_dot...
                + (1-self.beta)*((h - self.h_d1) / self.Ts);
            self.h_d1 = h;            
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