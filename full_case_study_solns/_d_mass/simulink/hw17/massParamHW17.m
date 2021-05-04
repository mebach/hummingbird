addpath('../.')
mass_param

% sample time for controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.1;

% input constraint
P.Fmax = 2;

% tuning parameters
%tr = 5;   % first part
tr = 2.5;  % tuned value of the rise time
zeta = 0.707;
P.ki = 0.5; % integrator gain

% PD gains
% open loop char polynomial and poles
Delta_ol = [1,P.b/P.m,P.k/P.m];
wn = 2.2/tr;
Delta_cl_d = [1,2*zeta*wn,wn^2];
P.kp =  P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));

% transfer function for robot arm
Plant = tf([1/P.m],[1, P.b/P.m, P.k/P.m]);
C_pid = tf([(P.kd+P.kp*P.sigma),(P.kp+P.ki*P.sigma),P.ki],[P.sigma,1,0]);

% margin and bode plots 
figure(1), clf, margin(Plant*C_pid), grid on, hold on
bode(Plant*C_pid/(1+Plant*C_pid)) 
legend('Open Loop', 'Closed Loop')





