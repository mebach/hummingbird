A = P.m2*P.length^2/3 + P.m1*P.z0^2;

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

Gth_cl = tf([P.kp_th*P.length/A],[1 P.kd_th*P.length/A P.kp_th*P.length/A])
Gz = tf(-P.g,[1 0 0]);

% Transfer function from F to z with inner loops closed
G = Gth_cl*Gz

% DC gain for inner loop
k_DC_th = 1;

% PD design for outer loop using root locus
wn_z     = 2.2/tr_z; % natural frequency - outer loop

% A good PD design
K = -0.027;
a = 0.36;

% PD design with frequency of inner-loop poles "maximized"
K = -0.0308;
a = 0.389;

C = tf([1/a 1],1);

CLTF = minreal(K*C*G/(1+K*C*G));
CLTF2 = minreal(K*G/(1+K*C*G));

figure(1); clf;
rlocus(-C*G); hold on;
sgrid(0.7,[wn_z 100]);
axis([-4 1 -2.5 2.5]);
rlocus(C*G,K,'r^'); hold off;

figure(2); clf;
step(CLTF);

figure(3); clf;
step(CLTF2);

% Compute PD gains
P.kp_z   = K; % kp - outer
P.kd_z   = K/a; % kd - outer

