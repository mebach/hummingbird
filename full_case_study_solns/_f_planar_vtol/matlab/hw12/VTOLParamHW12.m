% inverted ballbeam - parameter file for hw8
addpath ./.. % adds the parent directory to the path
VTOLParam % general ballbeam parameters

% turning parameters
wn_h    = 1;
zeta_h  = 0.707;
wn_z    = 0.9905;
zeta_z  = 0.707;
wn_th   = 13.3803;
zeta_th = 0.707;
integrator_h = -1;
integrator_z = -1;

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

% form augmented system
A1_lon = [A_lon, zeros(2,1); -C_lon, 0];
B1_lon = [B_lon; 0];
Cout_lat = [1, 0, 0, 0];
A1_lat = [A_lat, zeros(4,1); -Cout_lat, 0];
B1_lat = [B_lat; 0];

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

sprintf('K_lat: (%f, %f, %f, %f)\nki_lat: %f\nK_lon: (%f, %f)\nki_lon: %f\n',...
    P.K_lat(1), P.K_lat(2), P.K_lat(3), P.K_lat(4), P.ki_lat,...
    P.K_lon(1), P.K_lon(2), P.ki_lon)

