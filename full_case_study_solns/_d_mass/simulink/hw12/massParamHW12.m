% tuning parameters
tr = 2.5;  % tuned value of the rise time
zeta = 0.707;
integrator_pole = -10;

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

% form augmented system
A1 = [A, zeros(2,1); -C, 0];
B1 = [B; 0];


% gain calculation
wn = 2.2/tr;
ol_char_poly=charpoly(A1);
des_char_poly = conv([1,2*zeta*wn,wn^2],...
                     poly(integrator_pole));
des_poles = roots(des_char_poly);
% is the system controllable?
if rank(ctrb(A1,B1))~=3, 
    disp('System Not Controllable'); 
else % if so, compute gains
    K1   = place(A1,B1,des_poles); 
    P.K  = K1(1:2);
    P.ki = K1(3);
end


