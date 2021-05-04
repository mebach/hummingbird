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
B1 = [P.B; 0];

% gain calculation
wn = 2.2/tr;
ol_char_poly=charpoly(A1);
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A1,B1))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end

% observer design
% form augmented system for disturbance observer
A2 = [P.A, P.B; zeros(1,2), 0];
C2 = [P.C, 0];
% pick observer poles
wn_obs = 2.2/tr_obs;
des_obs_poles = roots(conv(...
                [1, 2*zeta*wn_obs, wn_obs^2],...
                poly(dist_pole)));
                
% is the system observable?
if rank(obsv(A2,C2))~=3
    disp('System Not Observable'); 
else % if so, compute gains
    L2 = place(A2',C2',des_obs_poles)'; 
    P.L = L2(1:2);
    P.Ld = L2(3);
end

