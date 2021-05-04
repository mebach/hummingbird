function out=VTOL_ctrl(in,P)
    h_r     = in(1);
    z_r     = in(2);
    z_m     = in(3);
    h_m     = in(4);
    theta_m = in(5);
    t       = in(6);

    % equilibrium force
    Fe = (P.mc+2*P.mr)*P.g; 

    % implement observer
    persistent xhat_lon       % estimated state (for observer)
    persistent F
    persistent xhat_lat
    persistent tau
    if t<P.Ts,
        xhat_lon = [0;0];
        F    = Fe;
        xhat_lat = [0;0;0;0];
        tau  = 0;
    end
    N = 10;
    for i=1:N,
        xhat_lon = xhat_lon + ...
            P.Ts/N*(P.A_lon*xhat_lon+P.B_lon*(F-Fe)...
                    + P.L_lon*(h_m-P.C_lon*xhat_lon));

        xhat_lat = xhat_lat + ...
            P.Ts/N*(P.A_lat*xhat_lat+P.B_lat*tau...
                  +P.L_lat*([z_m;theta_m]-P.C_lat*xhat_lat));
    end
    hhat = xhat_lon(1);
    zhat = xhat_lat(1);
        
    % integrator on altitude error
    error_h = h_r - hhat;
    persistent integrator_h
    persistent error_h_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator_h  = 0;
        error_h_d1    = 0;
    end
    integrator_h = integrator_h...
                       + (P.Ts/2)*(error_h+error_h_d1);
    error_h_d1 = error_h;

    % integrator on position error
    error_z = z_r - zhat;
    persistent integrator_z
    persistent error_z_d1
    % reset persistent variables at start of simulation
    if t<P.Ts==1,
        integrator_z  = 0;
        error_z_d1    = 0;
    end
    integrator_z = integrator_z...
                       + (P.Ts/2)*(error_z+error_z_d1);
    error_z_d1 = error_z;


    % longitudinal control for alititude
    % compute the state feedback controller
    F = Fe -P.K_lon*xhat_lon - P.ki_lon*integrator_h;

    % lateral control for position
    % compute the state feedback controller
    tau = -P.K_lat*xhat_lat - P.ki_lat*integrator_z;

    % produce forces on right and left rotors
    u = P.mixing*[F; tau];

    % x = [z; h; theta; zdot; hdot; thetadot]
    out = [u;...
            xhat_lat(1); xhat_lon(1); xhat_lat(2);...
            xhat_lat(3); xhat_lon(2); xhat_lat(4)];

end

% saturation function
function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end