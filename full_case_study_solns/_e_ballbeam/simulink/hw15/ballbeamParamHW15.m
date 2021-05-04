% ballbeam parameter file
addpath ./.. % adds the parent directory to the path
ballbeam_param % general parameters

% parameter for dirty derivative
P.sigma = 0.05;

P_in = tf([P.ell/(P.m2*P.ell^2/3+P.m1*(P.ell/2)^2)],[1,0,0]);
P_out = tf(-P.g,[1,0,0]);

figure(2), clf
bode(P_in)
title('Ballbeam - Inner Loop')
grid on

figure(3), clf
bode(P_out)
title('Ballbeam - Outer Loop')
grid on