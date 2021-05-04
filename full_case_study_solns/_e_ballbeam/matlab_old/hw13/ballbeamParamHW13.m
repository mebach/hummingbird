% inverted ballbeam - parameter file for hw8
addpath ./.. % adds the parent directory to the path
ballbeamParam % general ballbeam parameters

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
wn_th_obs   = 5*2.2/tr_th;
wn_z_obs    = 5*2.2/tr_z;


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
des_obsv_char_poly = conv([1,2*zeta_z*wn_z_obs,wn_z_obs^2],...
                      [1,2*zeta_th*wn_th_obs,wn_th_obs^2]);
des_obsv_poles = roots(des_obsv_char_poly);

% is the system observable?
if rank(obsv(P.A,P.C))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L = place(P.A', P.C', des_obsv_poles)';
end


sprintf('K: [%f, %f, %f, %f]\nki: %f\nL: [%f, %f, %f, %f]^T',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki, P.L(1), P.L(2), P.L(3), P.L(4))

