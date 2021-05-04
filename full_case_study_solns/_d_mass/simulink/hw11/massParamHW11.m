% tuning parameters
tr = 2;         % rise time from D.8
% tr = 2.5;       % slower rise time to avoid saturation
zeta = 0.7;

% sample time for controller
P.Ts = 0.01;

% dirty derivative gain
P.sigma = 0.05;
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% input constraint
P.Fmax = 2;

% state space design 
A = [...
    0, 1;...
    -P.k/P.m, -P.b/P.m;...
    ];
B = [0; 1/P.m ];
C = [...
    1, 0;...
    ];

% gain calculation
wn = 2.2/tr;
ol_char_poly = charpoly(A);
des_char_poly = [1,2*zeta*wn,wn^2];
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A,B))~=2, disp('System Not Controllable'); end
P.K = place(A,B,des_poles);
P.kr = -1/(C*inv(A-B*P.K)*B);


