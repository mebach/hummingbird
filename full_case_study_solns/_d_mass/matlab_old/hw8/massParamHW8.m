% mass - parameter file
addpath ./.. % adds the parent directory to the path
massParam % general parameters

% tuning parameters
tr = 2.2;  % tuned value of the rise time
zeta = 0.7;

% dirty derivative gain
P.sigma = 0.05;

% input constraint
P.F_max = 2;

% open loop char polynomial and poles
Delta_ol = [1,P.b/P.m,P.k/P.m];
p_ol = roots(Delta_ol);

% PD gains
wn = 2.2/tr;
Delta_cl_d = [1,2*zeta*wn,wn^2];
P.kp =  P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));

fprintf('\t kp: %f\n', P.kp)
fprintf('\t kd: %f\n', P.kd)

