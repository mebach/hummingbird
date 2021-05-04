% VTOL parameter file
addpath ./.. % adds the parent directory to the path
VTOL_param % general parameters

% load parameters from HW 10
addpath ./../hw10
VTOLParamHW10

P_lon = tf(1/(P.mc+2*P.mr), [1,0,0]);
P_lat_in = tf(1/(P.Jc+2*P.mr*P.d^2), [1,0,0]);
P_lat_out = tf(-P.Fe/(P.mc+2*P.mr), [1,P.mu/(P.mc+2*P.mr),0]);

C_lon = tf([(P.kd_h+P.kp_h*P.sigma),(P.kp_h+P.ki_h*P.sigma),P.ki_h],[P.sigma,1,0]);

C_lat_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_lat_out = tf([(P.kd_z+P.sigma*P.kp_z), P.kp_z], [P.sigma, 1]);

% margin and bode plots for longitudinal controller 
figure(2), clf, margin(P_lon*C_lon), grid on, hold on
bode(P_lon*C_lon/(1+P_lon*C_lon)) 
legend('Open Loop-Inner', 'Closed Loop-Inner')

% margin and bode plots for lateral controller 
figure(3), clf, margin(P_lat_in*C_lat_in), grid on, hold on
bode(P_lat_in*C_lat_in/(1+P_lat_in*C_lat_in)) 
margin(P_lat_out*C_lat_out)
bode(P_lat_out*C_lat_out/(1+P_lat_out*C_lat_out))
legend('Open Loop-Inner', 'Closed Loop-Inner',...
       'Open Loop-Outer', 'Closed Loop-Outer')


