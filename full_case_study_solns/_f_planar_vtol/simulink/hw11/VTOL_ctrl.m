 function u=VTOL_ctrl(in,P)
    h_r   = in(1);
    z_r   = in(2);
    z     = in(3);
    h     = in(4);
    theta = in(5);
    t     = in(6);
    
    % use a digital differentiator to find hdot, zdot and thetadot
    persistent hdot
    persistent h_d1
    persistent zdot
    persistent z_d1
    persistent thetadot
    persistent theta_d1
    % reset persistent variables at start of simulation
    if t<P.Ts
        hdot        = 0;
        h_d1        = 0;
        zdot        = 0;
        z_d1        = 0;
        thetadot    = 0;
        theta_d1    = 0;
    end
    hdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*hdot...
        + 2/(2*P.sigma+P.Ts)*(h-h_d1);
    zdot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*zdot...
        + 2/(2*P.sigma+P.Ts)*(z-z_d1);
    thetadot = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts)*thetadot...
        + 2/(2*P.sigma+P.Ts)*(theta-theta_d1);
    h_d1 = h;
    z_d1 = z;
    theta_d1 = theta;

    % longitudinal control for altitude
    % construct the state
    x_lon = [h; hdot];
    % equilibrium force
    Fe = (P.mc+2*P.mr)*P.g/cos(theta); 
        % divide Fe by cos(theta) so that force is right during
        % lateral translations.
    % compute the state feedback controller
    F_tilde = -P.K_lon*x_lon + P.kr_lon*h_r;
    F = Fe + F_tilde;

    % lateral control for position
    % construct the state
    x_lat = [z; theta; zdot; thetadot];
    % compute the state feedback controller
    tau = -P.K_lat*x_lat + P.kr_lat*z_r;

    % produce forces on right and left rotors
    u = P.mixing*[F; tau];

end

%-----------------------------------------------------------------
% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end