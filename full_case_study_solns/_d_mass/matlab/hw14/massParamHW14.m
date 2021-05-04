% mass - parameter file
addpath ./.. % adds the parent directory to the path
massParam % general parameters

% tuning parameters
tr = 2.5;  % tuned value of the rise time
zeta = 0.707;
integrator_pole = -5;  % integrator pole
tr_obs = tr/10;  % rise time for observer
dist_pole = -1; % pole for disturbance observer


% sample time for controller
P.Ts = 0.01;

% input constraint
P.Fmax = 2;

% state space design 
P.A = [...
    0, 1;...
    -P.k/P.m, -P.b/P.m;...
    ];
P.B = [0; 1/P.m ];
P.C = [...
    1, 0;...
    ];

% form augmented system
A1 = [P.A, zeros(2,1); -P.C, 0];
P.B1 = [P.B; 0];

% gain calculation
wn = 2.2/tr;
ol_char_poly=charpoly(A1);
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A1, P.B1))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1, P.B1, des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% observer design
% form augmented system for disturbance observer
P.A2 = [P.A, P.B; zeros(1,2), 0];
P.C2 = [P.C, 0];
% pick observer poles
wn_obs = 2.2/tr_obs;
des_obs_poles = roots(conv(...
                [1, 2*zeta*wn_obs, wn_obs^2],...
                poly(dist_pole)));
                
% is the system observable?
if rank(obsv(P.A2, P.C2))~=3
    disp('System Not Observable'); 
else % if so, compute gains
    P.L2 = place(P.A2', P.C2', des_obs_poles)'; 
    L = P.L2(1:2);
    Ld = P.L2(3);
end

fprintf('\t K: [%f, %f]\n', P.K(1), P.K(2))
fprintf('\t ki: %f\n', P.ki)
fprintf('\t L^T: [%f, %f]\n', L(1), L(2))
fprintf('\t Ld: %f\n', Ld)


