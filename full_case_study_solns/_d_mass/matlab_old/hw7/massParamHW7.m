addpath ./..
massParam

Delta_ol = [1,P.b/P.m,P.k/P.m];
p_ol = roots(Delta_ol);

Delta_cl_d = poly([-1,-1.5]);

P.kp = P.m*(Delta_cl_d(3)-Delta_ol(3));
P.kd = P.m*(Delta_cl_d(2)-Delta_ol(2));
