% ballbeam parameter file
addpath ./.. % adds the parent directory to the path
ballbeam_param % general parameters

% load parameters from HW 10
addpath ./../hw10
ballbeamParamHW10

P_in = tf([b0],[1,0,0]);
P_out = tf(-P.g,[1,0,0]);

C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],[P.sigma,1,0]);


figure(2), clf, 
bode(P_in), grid on
hold on
bode(series(C_in,P_in))
legend('No control', 'PD')
title('Ballbeam, Inner Loop')

figure(3), clf, 
bode(P_out), grid on
hold on
bode(series(C_out,P_out))
legend('No control', 'PID')
title('Ballbeam, Outer Loop')

