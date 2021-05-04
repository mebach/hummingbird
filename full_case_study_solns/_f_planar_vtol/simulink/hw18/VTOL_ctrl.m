function u=VTOL_ctrl(in,P)
    h_r     = in(1);
    z_r     = in(2);
    z_m     = in(3);
    h_m     = in(4);
    theta_m = in(5);
    t       = in(6);

    % equilibrium force
    Fe = (P.mc+2*P.mr)*P.g; 

    % implement observer
    persistent x_lon_C
    persistent x_lon_F
    persistent x_lat_in_C
    persistent x_lat_out_C
    persistent x_lat_out_F
    if t<P.Ts,
        x_lon_C = zeros(size(P.A_lon_C,1),1);
        x_lon_F = zeros(size(P.A_lon_F,1),1);
        x_lat_in_C = zeros(size(P.A_lat_in_C,1),1);
        x_lat_out_C = zeros(size(P.A_lat_out_C,1),1);
        x_lat_out_F = zeros(size(P.A_lat_out_F,1),1);
    end
        
    %--------------------------
    % longitudinal controller
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        x_lon_F = x_lon_F + P.Ts/N*( P.A_lon_F*x_lon_F + P.B_lon_F*h_r );
        % output equation for the prefilter
        h_r_filtered = P.C_lon_F*x_lon_F + P.D_lon_F*h_r;
        % error signal for longitudinal loop
        error_h = h_r_filtered - h_m;
        x_lon_C = x_lon_C + P.Ts/N*( P.A_lon_C*x_lon_C + P.B_lon_C*error_h );
        % output equation for the controller
        F = Fe + P.C_lon_C*x_lon_C + P.D_lon_C*error_h;
    end
    
    
    %--------------------------
    % lateral controller
     N = 10; % number of Euler integration steps for each sample
    for i=1:N
        x_lat_out_F = x_lat_out_F + P.Ts/N*( P.A_lat_out_F*x_lat_out_F + P.B_lat_out_F*z_r );
        % output equation for the prefilter
        z_r_filtered = P.C_lat_out_F*x_lat_out_F + P.D_lat_out_F*z_r;
        % error signal for longitudinal loop
        error_z = z_r_filtered - z_m;
        x_lat_out_C = x_lat_out_C + P.Ts/N*( P.A_lat_out_C*x_lat_out_C + P.B_lat_out_C*error_z );
        % output equation for the controller
        theta_r = P.C_lat_out_C*x_lat_out_C + P.D_lat_out_C*error_z;
        error_theta = theta_r - theta_m;
        x_lat_in_C = x_lat_in_C + P.Ts/N*( P.A_lat_in_C*x_lat_in_C + P.B_lat_in_C*error_theta );
        tau = P.C_lat_in_C*x_lat_in_C + P.D_lat_in_C*error_theta;
    end
   
    %--------------------------------------
    % Mix two controllers to produce forces on right and left rotors
    u = P.mixing*[F; tau];

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end