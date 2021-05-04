% tuning parameters
tr_h    = 8;        % rise time from F.8
tr_h    = tr_h/4;   % faster!
wn_h    = 2.2/tr_h;
zeta_h  = 0.707;

tr_z    = 8;        % rise time from F.8
tr_z    = tr_z/4;   % faster!
wn_z    = 2.2/tr_z;
zeta_z  = 0.707;

tr_th   = tr_z/10;
wn_th   = 2.2/tr_th;
zeta_th = 0.707;

% dirty derivative gain for differentiator
P.sigma = 0.05;

% mixing matrix
P.mixing = inv([1, 1; P.d, -P.d]);

% equilibrium force and constraints
P.Fe = (P.mc+2*P.mr)*P.g;
P.Ftildemax = 2*P.fmax - P.Fe;
P.taumax = (P.fmax-P.Fe/2)/P.d;

% state space design
A_lon = [...
    0, 1;...
    0, 0;...
    ];
B_lon = [0; 1/(P.mc+2*P.mr)];
C_lon = [1, 0];
A_lat = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -(P.Fe/(P.mc+2*P.mr)), -(P.mu/(P.mc+2*P.mr)), 0;...
    0, 0, 0, 0;...
    ];
B_lat = [0;0;0;1/(P.Jc+2*P.mr*P.d^2)];
C_lat = [1, 0, 0, 0; 0, 1, 0, 0];

% gain calculation
ol_char_poly_lon = charpoly(A_lon);
des_char_poly_lon = [1,2*zeta_h*wn_h,wn_h^2];
des_poles_lon = roots(des_char_poly_lon);

ol_char_poly_lat = charpoly(A_lat);
des_char_poly_lat = conv([1,2*zeta_z*wn_z,wn_z^2],...
                     [1,2*zeta_th*wn_th,wn_th^2]);
des_poles_lat = roots(des_char_poly_lat);

% gains for longitudinal system
if rank(ctrb(A_lon,B_lon))~=2, 
	disp('Lon System Not Controllable');
end
P.K_lon = place(A_lon,B_lon,des_poles_lon);
P.kr_lon = -1/(C_lon*inv(A_lon-B_lon*P.K_lon)*B_lon);

% gains for lateral system
if rank(ctrb(A_lat,B_lat))~=4, 
	disp('Lat System Not Controllable'); 
end
P.K_lat = place(A_lat,B_lat,des_poles_lat);
Cout = [1, 0, 0, 0];
P.kr_lat = -1/(Cout*inv(A_lat-B_lat*P.K_lat)*B_lat);

% P.K_lon
% P.kr_lon
% P.K_lat
% P.kr_lat

