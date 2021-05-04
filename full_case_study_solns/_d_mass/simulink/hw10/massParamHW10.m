% sample time for controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% input constraint
P.Fmax = 2;

% tuning parameters
%tr = 5;   % first part
tr = 2.5;  % tuned value of the rise time
zeta = 0.707;
P.ki = 1.5; % integrator gain

% PD gains
% open loop char polynomial and poles
Delta_ol = [1,P.b/P.m,0];
wn = 2.2/tr;
Delta_cl_d = [1,2*zeta*wn,wn^2];
P.kp =  P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));

