% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% equilibrium position
P.ze = P.length/2;

% tuning parameters
tr_z = 1.2; % tuned for fastest rise time without saturation
tr_th = 0.5; % rise time for angle
zeta_z = 0.707; % damping ratio for outer loop
zeta_th  = 0.707; % damping ratio for inner loop
integrator_pole = -5;
% pick observer poles
wn_th_obs   = 8*wn_th;
wn_z_obs    = 5*wn_z;
dist_obsv_pole = -10;

% state space design
P.A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.m1*P.g/((P.m2*P.length^2)/3+P.m1*(P.length/2)^2), 0, 0;...
    -P.g, 0, 0, 0;...
];
P.B = [0; 0; P.length/(P.m2*P.length^2/3+P.m1*P.length^2/4); 0 ];
P.C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% form augmented system
Cout = [0,1,0,0];
A1 = [P.A, zeros(4,1); -Cout, 0];
B1 = [P.B; 0];

% compute gains
wn_z     = 2.2/tr_z;
wn_th    = 2.2/tr_th; 
ol_char_poly = charpoly(A1);
des_char_poly = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]),...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);

% is the system controllable?
if rank(ctrb(A1,B1))~=5
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:4);
    P.ki = K1(5);
end

% observer design
% form augmented system for disturbance observer
A2 = [P.A, P.B; zeros(1,4), zeros(1,1)];
C2 = [P.C, zeros(2,1)];
des_obsv_char_poly = conv(conv(...
                      [1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]),...
                      poly(dist_obsv_pole));
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(A2,C2))~=5
    disp('System Not Observable'); 
else % if so, compute gains
    L2 = place(A2', C2', des_obsv_poles)';
    P.L = L2(1:4,:);
    P.Ld = L2(5,:);
end

sprintf('K:\t[%f, %f, %f, %f]\nki:\t%f\nL^T:\t[%f, %f, %f, %f;\n\t %f, %f, %f, %f]\nLd:\t[%f, %f]',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki,...
    P.L(1,1), P.L(2,1), P.L(3,1), P.L(4,1), P.L(1,2), P.L(2,2), P.L(3,2), P.L(4,2),...
    P.Ld(1), P.Ld(2))

