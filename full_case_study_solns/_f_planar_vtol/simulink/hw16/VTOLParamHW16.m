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

figure(2), clf, 
bode(P_lon), grid on
hold on
bode(series(C_lon,P_lon))
legend('No control', 'PID')
title('VTOL Longitudinal Loop')


figure(3), clf
bode(P_lat_in)
hold on
bode(series(C_lat_in,P_lat_in))
legend('P', 'PC')
title('VTOL Lateral Inner Loop')
grid on

figure(4), clf
bode(P_lat_out)
hold on
bode(series(C_lat_out,P_lat_out))
legend('P', 'PC')
title('VTOL Lateral Outer Loop')
grid on



