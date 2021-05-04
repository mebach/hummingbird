% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% tunning parameters
%tr_z = 10;  % rise time for outer loop - first part of problem
tr_z = 1.2; % tuned for fastest rise time without saturation
zeta_z = 0.707; % damping ratio for outer loop
M = 10;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop
P.ki_z = -0.1; % integral gain on outer loop

% PD design for inner loop
P.ze = P.length/2;  % equilibrium position - center of beam
b0 = P.length/(P.m2*P.length^2/3+P.m1*P.ze^2);
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
P.kp_th  = wn_th^2/b0 % kp - inner
P.kd_th  = 2*zeta_th*wn_th/b0; % kd - inner

% DC gain for inner loop
k_DC_gain = 1;

% PD design for outer loop
wn_z     = 2.2/tr_z; % natural frequency - outer loop
P.kp_z   = -wn_z^2/P.g; % kp - outer
P.kd_z   = -2*zeta_z*wn_z/P.g; % kd - outer

sprintf('DC_gain: %f\nkp_th: %f\nkd_th: %f\nkp_z: %f\nkd_z: %f\nki_z: %f\n',...
    k_DC_gain, P.kp_th, P.kd_th, P.kp_z, P.kd_z, P.ki_z)

