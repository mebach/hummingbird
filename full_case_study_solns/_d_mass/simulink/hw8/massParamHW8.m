% tuning parameters
tr = 2.0;   % tuned value of the rise time
% tr = 1.639;  % value of rise time just causes saturation for F_max = 6 N
zeta = 0.7;

% dirty derivative gain
P.sigma = 0.05;

% input constraint
P.F_max = 6;

% open loop char polynomial and poles
Delta_ol = [1,P.b/P.m,P.k/P.m];
p_ol = roots(Delta_ol);

% PD gains
wn = 2.2/tr;
Delta_cl_d = [1,2*zeta*wn,wn^2];
P.kp =  P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));
