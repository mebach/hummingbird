% dirty derivative parameters
P.sigma = 0.001; % cutoff freq for dirty derivative

% tuning parameters
tr_z = 5;  % rise time for outer loop - first part of problem
%tr_z = 1.7; % tuned for fastest rise time without saturation
zeta_z = 0.707; % damping ratio for outer loop
M = 5;    % time scale separation between inner and outer loop
zeta_th  = 0.707; % damping ratio for inner loop

% PD design for inner loop
P.ze = P.length/2;  % equilibrium position - center of beam
b0 = P.length/(P.m2*P.length^2/3+P.m1*P.ze^2);
tr_theta = tr_z/M;  % rise time for inner loop
wn_th    = 2.2/tr_theta; % natural frequency for inner loop
P.kp_th  = wn_th^2/b0; % kp - inner
P.kd_th  = 2*zeta_th*wn_th/b0; % kd - inner

% DC gain for inner loop
k_DC_th = 1;

% PD design for outer loop
wn_z     = 2.2/tr_z; % natural frequency - outer loop
P.kp_z   = -wn_z^2/P.g; % kp - outer
P.kd_z   = -2*zeta_z*wn_z/P.g; % kd - outer

fprintf('\t DC_gain: %f\n', k_DC_th)
fprintf('\t kp_th: %f\n', P.kp_th)
fprintf('\t kd_th: %f\n', P.kd_th)
fprintf('\t kp_z: %f\n', P.kp_z)
fprintf('\t kd_z: %f\n', P.kd_z)

