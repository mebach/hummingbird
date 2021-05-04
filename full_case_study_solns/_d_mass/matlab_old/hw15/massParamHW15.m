addpath('../.')
massParam

% transfer function for robot arm
G = tf([1/P.m],[1, P.b/P.m, P.k/P.m]);
figure(1), clf, bode(G), grid on

