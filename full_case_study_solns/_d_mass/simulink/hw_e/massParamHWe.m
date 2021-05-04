% tuning parameters
tr = 2.2;  % tuned value of the rise time
zeta = 0.7;

% open loop char polynomial and poles
Delta_ol = [1,P.b/P.m,P.k/P.m];
p_ol = roots(Delta_ol);

% PD gains
wn = 2.2/tr;
Delta_cl_d = [1,2*zeta*wn,wn^2];
P.kp =  P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));

% Express closed-loop characteristic equation in Evan's form
% with ki as the variable of interest
L = tf([1/P.m],[1, (P.b+P.kd)/P.m, (P.k+P.kp)/P.m,0]);
figure(1); clf; 
rlocus(L);