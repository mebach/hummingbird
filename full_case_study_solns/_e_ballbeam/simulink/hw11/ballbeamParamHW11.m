% dirty derivative parameters
P.sigma = 0.05; % cutoff freq for dirty derivative
P.beta = (2*P.sigma-P.Ts)/(2*P.sigma+P.Ts); % dirty derivative gain

% equilibrium position
P.ze = P.length/2;

% tuning parameters
% tr_z = 1.2; % tuned for fastest rise time without saturation
% tr_th = 0.5; % rise time for angle
tr_z = 10;  % rise time for z from E.8
tr_th = 1;  % rise time for theta from E.8

zeta_z = 0.707; % damping ratio for outer loop
zeta_th  = 0.707; % damping ratio for inner loop


% state space design
A = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -P.m1*P.g/((P.m2*P.length^2)/3+P.m1*(P.length/2)^2), 0, 0;...
    -P.g, 0, 0, 0;...
];
B = [0; 0; P.length/(P.m2*P.length^2/3+P.m1*P.length^2/4); 0 ];
C = [...
    1, 0, 0, 0;...
    0, 1, 0, 0;...
    ];

% compute gains
wn_z     = 2.2/tr_z;
wn_th    = 2.2/tr_th; 
ol_char_poly = charpoly(A);
des_char_poly = conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]);
des_poles = roots(des_char_poly);
% Compute the gains if the system is controllable
if rank(ctrb(A, B)) ~= 4
    disp('The system is not controllable')
else
    P.K = place(A, B, des_poles);
    Cout = [0, 1, 0, 0];
    P.kr = -1/(Cout*inv(A-B*P.K)*B);
end

sprintf('K: (%f, %f, %f, %f)\nkr: %f\n', P.K(1), P.K(2), P.K(3), P.K(4), P.kr)

