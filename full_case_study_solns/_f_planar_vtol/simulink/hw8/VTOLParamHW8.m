
% tuning parameters
tr_h = 8;  % rise time for altitude - original
% tr_h = 3; % turned for fastest performance without saturation
zeta_h = 0.707; % damping ratio for altitude
tr_z = 8; % rise time for outer lateral loop (position) - original
% tr_z = 3; % tuned for fastest performance without saturation
M = 10; % time separation between inner and outer lateral loops
zeta_z = 0.707; % damping ratio for outer lateral loop
zeta_th = 0.707; % damping ratio for inner lateral loop


% PD gains for longitudinal (altitude) control
wn_h = 2.2/tr_h;  % natural frequency
Delta_cl_d = [1, 2*zeta_h*wn_h, wn_h^2]; % desired closed loop char eq
P.kp_h = Delta_cl_d(3)*(P.mc+2*P.mr); % kp - altitude
P.kd_h = Delta_cl_d(2)*(P.mc+2*P.mr); % kd = altitude
P.Fe = (P.mc+2*P.mr)*P.g; % equilibrium force

% PD gains for lateral inner loop
b0       = 1/(P.Jc+2*P.mr*P.d^2);
tr_th    = tr_z/M;
wn_th    = 2.2/tr_th;
P.kp_th  = wn_th^2/b0;
P.kd_th  = 2*zeta_th*wn_th/b0;

% DC gain for lateral inner loop
k_DC_th = 1;

%PD gain for lateral outer loop
b1       = -P.Fe/(P.mc+2*P.mr);
a1       = P.mu/(P.mc+2*P.mr);
wn_z     = 2.2/tr_z;
P.kp_z   = wn_z^2/b1;
P.kd_z   = (2*zeta_z*wn_z-a1)/b1;