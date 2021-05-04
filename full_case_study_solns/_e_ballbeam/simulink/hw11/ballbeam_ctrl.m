function F=ballbeam_ctrl(in,P)
    z_r   = in(1);
    z     = in(2);
    theta = in(3);
    t     = in(4);
    
     % use a digital differentiator to find zdot and thetadot
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        zdot        = 0;
        z_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    zdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    z_d1 = z;
    theta_d1 = theta;

    % construct the state
    z_e = P.length/2;
    % NOTE:  remember the feedback control should actually be
    % u_tilde = -K*x_tilde + kr*z_r_tilde
    % since the only value that deviates from zero is z, we have
    x = [theta; z; thetadot; zdot];
    x_e = [0; z_e; 0; 0];
    % equilibrium force
    F_e = 0.5*P.m2*P.g + P.m1*P.g*z_e/P.length;
    % compute the state feedback controller
    F_tilde = - P.K*(x-x_e) + P.kr*(z_r-z_e);
    F = sat( F_e + F_tilde, P.Fmax);
end



%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end