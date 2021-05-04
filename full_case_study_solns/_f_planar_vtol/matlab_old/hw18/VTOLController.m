classdef VTOLController < handle
    %----------------------------
    properties
        Fe
        x_lon_C
        x_lon_F
        A_lon_C
        B_lon_C
        C_lon_C
        D_lon_C
        A_lon_F
        B_lon_F
        C_lon_F
        D_lon_F
        
        x_lat_in_C
        A_lat_in_C
        B_lat_in_C
        C_lat_in_C
        D_lat_in_C

        x_lat_out_C
        x_lat_out_F
        A_lat_out_C
        B_lat_out_C
        C_lat_out_C
        D_lat_out_C
        A_lat_out_F
        B_lat_out_F
        C_lat_out_F
        D_lat_out_F
        limit
        Ts
        N
    end
    %----------------------------
    methods
        %--Constructor--------------------------
        function self = VTOLController(P)
            self.Fe = P.Fe;
            % initialized object properties
            self.x_lon_C = zeros(size(P.A_lon_C,1),1);
            self.x_lon_F = zeros(size(P.A_lon_F,1),1); 
            self.A_lon_C = P.A_lon_C;
            self.B_lon_C = P.B_lon_C;
            self.C_lon_C = P.C_lon_C;
            self.D_lon_C = P.D_lon_C;
            self.A_lon_F = P.A_lon_F;
            self.B_lon_F = P.B_lon_F;
            self.C_lon_F = P.C_lon_F;
            self.D_lon_F = P.D_lon_F;

            self.x_lat_in_C  = zeros(size(P.A_lat_in_C,1),1);
            self.A_lat_in_C = P.A_lat_in_C;
            self.B_lat_in_C = P.B_lat_in_C;
            self.C_lat_in_C = P.C_lat_in_C;
            self.D_lat_in_C = P.D_lat_in_C;

            self.x_lat_out_C = zeros(size(P.A_lat_out_C,1),1);
            self.x_lat_out_F = zeros(size(P.A_lat_out_F,1),1); 
            self.A_lat_out_C = P.A_lat_out_C;
            self.B_lat_out_C = P.B_lat_out_C;
            self.C_lat_out_C = P.C_lat_out_C;
            self.D_lat_out_C = P.D_lat_out_C;
            self.A_lat_out_F = P.A_lat_out_F;
            self.B_lat_out_F = P.B_lat_out_F;
            self.C_lat_out_F = P.C_lat_out_F;
            self.D_lat_out_F = P.D_lat_out_F;
            self.limit = P.fmax;
            self.Ts = P.Ts;
            self.N = 10; % number of Euler integration steps for each sample
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
            
            %-----Longitudinal Control------
            % solve differential equation defining prefilter
            self.updatePrefilterStateLon(h_r);
            h_r_filtered = self.C_lon_F*self.x_lon_F + self.D_lon_F*h_r;
                
            % error signal for longitudinal loop
            error_lon = h_r_filtered - h;
                
            % implement longitudinal controller
            self.updateControlStateLon(error_lon);
            F_tilde = self.C_lon_C*self.x_lon_C + self.D_lon_C*error_lon;
            
            %-----Lateral Control------
            %+++++Lateral Outer Loop+++++
            self.updatePrefilterStateLat(z_r);
            z_r_filtered = self.C_lat_out_F*self.x_lat_out_F + self.D_lat_out_F*z_r;
                
            % error signal for outer loop
            error_lat_out = z_r_filtered - z;
                
            % Outer loop control C_out
            self.updateControlStateLatOut(error_lat_out);
            theta_r = self.C_lat_out_C*self.x_lat_out_C + self.D_lat_out_C*error_lat_out;

            %+++++Lateral Inner Loop+++++
            % error signal for inner loop
            error_lat_in = theta_r - theta;
                
            % Inner loop control C_in
            self.updateControlStateLatIn(error_lat_in);
            tau = self.C_lat_in_C*self.x_lat_in_C + self.D_lat_in_C*error_lat_in;          

            % total force
            F = F_tilde + self.Fe;
            out = [F; tau];
        end
        %----------------------------
        function self = updatePrefilterStateLon(self, h_r)
            for i=1:self.N
                self.x_lon_F = self.x_lon_F + self.Ts/self.N*(... 
                    self.A_lon_F*self.x_lon_F + self.B_lon_F*h_r...
                    );
            end
        end
        %----------------------------
        function self = updateControlStateLon(self, error_lon)
            for i=1:self.N
                self.x_lon_C = self.x_lon_C + self.Ts/self.N*(...
                    self.A_lon_C*self.x_lon_C + self.B_lon_C*error_lon...
                    );
            end
        end
        %----------------------------
        function self = updatePrefilterStateLat(self, z_r)
            for i=1:self.N
                self.x_lat_out_F = self.x_lat_out_F + self.Ts/self.N*(... 
                    self.A_lat_out_F*self.x_lat_out_F + self.B_lat_out_F*z_r...
                    );
            end
        end
        %----------------------------
        function self = updateControlStateLatOut(self, error_lat_out)
            for i=1:self.N
                self.x_lat_out_C = self.x_lat_out_C + self.Ts/self.N*(...
                    self.A_lat_out_C*self.x_lat_out_C + self.B_lat_out_C*error_lat_out...
                    );
            end
        end
        %----------------------------
        function self = updateControlStateLatIn(self, error_lat_in)
            for i=1:self.N
                self.x_lat_in_C = self.x_lat_in_C + self.Ts/self.N*(...
                    self.A_lat_in_C*self.x_lat_in_C + self.B_lat_in_C*error_lat_in...
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