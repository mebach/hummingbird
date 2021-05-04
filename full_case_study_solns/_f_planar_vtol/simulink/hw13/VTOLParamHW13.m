% turning parameters
wn_h    = 1;
zeta_h  = 0.707;
wn_z    = 0.9905;
zeta_z  = 0.707;
wn_th   = 13.3803;
zeta_th = 0.707;
integrator_h = -3;
integrator_z = -2;

% equilibrium force and constraints
P.Fe = (P.mc+2*P.mr)*P.g;
%P.Ftildemax = 2*P.fmax - P.Fe;
%P.taumax = (P.fmax-P.Fe/2)/P.d;

% state space design
P.A_lon = [...
    0, 1;...
    0, 0;...
    ];
P.B_lon = [0; 1/(P.mc+2*P.mr)];
P.C_lon = [1, 0];
P.A_lat = [...
    0, 0, 1, 0;...
    0, 0, 0, 1;...
    0, -(P.Fe/(P.mc+2*P.mr)), -(P.mu/(P.mc+2*P.mr)), 0;...
    0, 0, 0, 0;...
    ];
P.B_lat = [0;0;0;1/(P.Jc+2*P.mr*P.d^2)];
P.C_lat = [1, 0, 0, 0; 0, 1, 0, 0];

% form augmented system
A1_lon = [P.A_lon, zeros(2,1); -P.C_lon, 0];
B1_lon = [P.B_lon; 0];
Cout_lat = [1, 0, 0, 0];
A1_lat = [P.A_lat, zeros(4,1); -Cout_lat, 0];
B1_lat = [P.B_lat; 0];


des_char_poly_lon = conv([1,2*zeta_h*wn_h,wn_h^2],...
                          poly(integrator_h));
des_poles_lon = roots(des_char_poly_lon);
des_char_poly_lat = conv(conv([1,2*zeta_z*wn_z,wn_z^2],...
                              [1,2*zeta_th*wn_th,wn_th^2]),...
                         poly(integrator_z));
des_poles_lat = roots(des_char_poly_lat);

% gains for longitudinal system
% is the system controllable?
if rank(ctrb(A1_lon,B1_lon))~=3
    disp('System Not Controllable'); 
else % if so, compute gains
    K1_lon   = place(A1_lon,B1_lon,des_poles_lon); 
    P.K_lon  = K1_lon(1:2);
    P.ki_lon = K1_lon(3);
end

% gains for lateral system
% is the system controllable?
if rank(ctrb(A1_lat,B1_lat))~=5
    disp('System Not Controllable'); 
else % if so, compute gains
    K1_lat   = place(A1_lat,B1_lat,des_poles_lat); 
    P.K_lat  = K1_lat(1:4);
    P.ki_lat = K1_lat(5);
end

% observer design
% pick observer poles
wn_h_obs    = 10*wn_h;
wn_z_obs    = 10*wn_z;
wn_th_obs   = 5*wn_th;
des_obsv_poles_lon = roots([1,2*zeta_h*wn_h_obs,wn_h_obs^2]);
des_obsv_poles_lat ...
    = [roots([1,2*zeta_z*wn_z_obs,wn_z_obs^2]);...
       roots([1,2*zeta_th*wn_th_obs,wn_th_obs^2])];


% is the longitudinal system observable?
if rank(obsv(P.A_lon,P.C_lon))~=2
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lon = place(P.A_lon', P.C_lon', des_obsv_poles_lon)';
end


% is the lateral system observable?
if rank(obsv(P.A_lat,P.C_lat))~=4
    disp('System Not Observable'); 
else % if so, compute gains
    P.L_lat = place(P.A_lat', P.C_lat', des_obsv_poles_lat)';
end






