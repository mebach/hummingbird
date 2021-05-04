% tuning parameters
tr = 2.2;  % tuned value of the rise time
zeta = 0.7;

% input constraint
P.F_max = 1000;

% open-loop transfer functions
nD_lead = [1/0.4 1];
dD_lead = [1/5 1];
D_lead = tf(nD_lead,dD_lead);

numG = 1/P.m;
denG = [1 P.b/P.m P.k/P.m];
G = tf(numG,denG);

K_lead = 3;

figure(1); clf;
rlocus(D_lead*G); hold on;
rlocus(D_lead*G,K_lead,'b^'); hold off;
sgrid(zeta,[tr/2.2 100]);
axis([-7 1 -4 4]);

CLTF1 = minreal(K_lead*D_lead*G/(1+K_lead*D_lead*G));

figure(2); clf;
step(CLTF1);

% Lag ratio of 20
nD_lag = [1/0.2 1];
dD_lag = [1/0.01 1];  
D_lag = tf(nD_lag,dD_lag);

D_leadlag = D_lead*D_lag;
[nD_leadlag,dD_leadlag] = tfdata(D_leadlag,'v');

K_leadlag = 65;
figure(3); clf;
rlocus(D_lead*D_lag*G); hold on;
rlocus(D_lead*D_lag*G,K_leadlag,'b^'); hold off;
sgrid(zeta,[tr/2.2 100]);
axis([-7 1 -4 4]);

CLTF2 = minreal(K_leadlag*D_lead*D_lag*G/(1+K_leadlag*D_lead*D_lag*G));

figure(4); clf;
step(CLTF2);




