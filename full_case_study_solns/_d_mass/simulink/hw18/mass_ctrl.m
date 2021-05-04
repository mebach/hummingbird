function out=mass_ctrl(in,P)
    z_r   = in(1);
    z_m   = in(2);
    t     = in(3);
    
    % initialize control and prefilter state z
    persistent z_C
    persistent z_F
    if t<P.Ts
          z_C = zeros(size(P.A_C,1),1);
          z_F = zeros(size(P.A_F,1),1);          
    end
    
    % prefilter the reference command
    % solve differential equation defining prefilter
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        z_F = z_F + P.Ts/N*( P.A_F*z_F + P.B_F*z_r );
    end
    % output equation for the prefilter
    z_r_filtered = P.C_F*z_F + P.D_F*z_r;
    
    % error signal
    error = z_r_filtered - z_m;

    % solve differential equation defining controller
    N = 10; % number of Euler integration steps for each sample
    for i=1:N
        z_C = z_C + P.Ts/N*( P.A_C*z_C + P.B_C*error );
    end
    % output equation for the controller
    F_tilde = P.C_C*z_C + P.D_C*error;
    
    % compute equilibrium force
    %F_e = P.k*z_m;
    %F_e = P.k*z_r_filtered;
    F_e = 0;
    % compute total force
    F = sat(F_e + F_tilde, P.Fmax); 
    
    out = [F];
    
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end