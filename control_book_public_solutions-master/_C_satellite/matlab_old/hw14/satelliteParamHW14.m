% satellite parameter file
addpath ./.. % adds the parent directory to the path
satelliteParam % general parameters

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       state feedback control with integrator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tuning parameters
wn_th   = 0.6;
zeta_th = 0.707;
wn_phi    = 1.1;
zeta_phi  = 0.707;
integrator_pole = -1;
% pick observer poles
wn_th_obs = 10*wn_th;
wn_phi_obs = 10*wn_phi;

% state space design
P.A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    -P.k/P.Js, P.k/P.Js, -P.b/P.Js, P.b/P.Js;...
    P.k/P.Jp, -P.k/P.Jp, P.b/P.Jp, -P.b/P.Jp;...

];
P.B = [0; 0; 1/P.Js; 0];
P.C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% form augmented system
Cout = [0,1,0,0];
A1 = [P.A, zeros(4,1); -Cout, 0];
B1 = [P.B; 0];

% compute gains
ol_char_poly = charpoly(P.A);
des_char_poly = conv(conv([1,2*zeta_th*wn_th,wn_th^2],...
                [1,2*zeta_phi*wn_phi,wn_phi^2]),...
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
% pick observer poles
wn_th_obs   = 10*wn_th;
wn_phi_obs    = 10*wn_phi;
dist_obsv_pole = -1;
des_obsv_char_poly = conv(conv(...
                [1,2*zeta_phi*wn_phi_obs,wn_phi_obs^2],...
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


sprintf('K: [%f, %f, %f, %f]\nki: %f\nL: [%f, %f, %f, %f]^T\nLd: [%f, %f]^T\n',...
    P.K(1), P.K(2), P.K(3), P.K(4), P.ki, P.L(1), P.L(2), P.L(3), P.L(4), P.Ld(1), P.Ld(2))







