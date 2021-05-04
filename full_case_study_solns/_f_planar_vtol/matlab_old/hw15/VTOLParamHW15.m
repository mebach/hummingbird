% VTOL parameter file
addpath ./.. % adds the parent directory to the path
VTOLParam % general parameters

P_lon = tf(1/(P.mc+2*P.mr), [1,0,0]);
P_lat_in = tf(1/(P.Jc+2*P.mr*P.d^2), [1,0,0]);
P_lat_out = tf(-P.Fe/(P.mc+2*P.mr), [1,P.mu/(P.mc+2*P.mr),0]);

figure(2), clf
bode(P_lon)
title('VTOL - Longitudinal')
grid on

figure(3), clf
bode(P_lat_in)
title('VTOL Lateral Inner Loop')
grid on

figure(4), clf
bode(P_lat_out)
title('VTOL Lateral Outer Loop')
grid on



