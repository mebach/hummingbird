function F=ballbeam_ctrl(in,P)
    z_r     = in(1);
    z_m     = in(2);
    theta_m = in(3);
    t       = in(4);

    % compute equilibrium values
    z_e = P.length/2;
    %F_e = 0.5*P.m2*P.g + P.m1*P.g*z_e/P.length;
    F_e = 0.5*P.m2*P.g + P.m1*P.g*z_m/P.length;
    %x_e = [0; z_e; 0; 0];
    
    % initialize controller state z
    persistent xout_C
    persistent xout_F
    persistent xin_C
    if t<P.Ts
          xout_C = zeros(size(P.Aout_C,1),1);
          xout_F = zeros(size(P.Aout_F,1),1); 
          xin_C  = zeros(size(P.Ain_C,1),1);
    end
    
    % prefilter the reference command for outer loop
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        xout_F = xout_F + P.Ts/N*( P.Aout_F*xout_F + P.Bout_F*z_r );
        % output equation for the prefilter
        z_r_filtered = P.Cout_F*xout_F + P.Dout_F*z_r;
        % error signal for outer loop
        error_out = z_r_filtered - z_m;
        xout_C = xout_C + P.Ts/N*( P.Aout_C*xout_C + P.Bout_C*error_out );
        % output equation for the controller
        theta_r = P.Cout_C*xout_C + P.Dout_C*error_out;

        % error signal for inner loop
        error_in = theta_r - theta_m;
        % state space equations for C
        xin_C = xin_C + P.Ts/N*( P.Ain_C*xin_C + P.Bin_C*error_in );
        % output equation for the controller
        F = sat(F_e+P.Cin_C*xin_C + P.Din_C*error_in, P.Fmax);
    end
  
end    


% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end