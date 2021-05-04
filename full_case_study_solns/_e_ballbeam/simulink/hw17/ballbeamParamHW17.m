% ballbeam parameter file
addpath ./.. % adds the parent directory to the path
ballbeam_param % general parameters

% load parameters from HW 10
addpath ./../hw10
ballbeamParamHW10

% transfer functions for plants and controllers
P_in = tf([b0],[1,0,0]);
P_out = tf(-P.g,[1,0,0]);
C_in = tf([(P.kd_th+P.sigma*P.kp_th), P.kp_th], [P.sigma, 1]);
C_out = tf([(P.kd_z+P.kp_z*P.sigma),(P.kp_z+P.ki_z*P.sigma),P.ki_z],[P.sigma,1,0]);

% margin and bode plots 
figure(2), clf, margin(P_in*C_in), grid on, hold on
bode(P_in*C_in/(1+P_in*C_in)) 
margin(P_out*C_out)
bode(P_out*C_out/(1+P_out*C_out))
legend('Open Loop-Inner', 'Closed Loop-Inner','Open Loop-Outer', 'Closed Loop-Outer')

