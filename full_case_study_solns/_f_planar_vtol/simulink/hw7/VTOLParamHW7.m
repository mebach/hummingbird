% select desired closed loop char eq
Delta_cl_d = poly([-.3,-.2]);

% PD gains
P.kp = Delta_cl_d(3)*(P.mc+2*P.mr);
P.kd = Delta_cl_d(2)*(P.mc+2*P.mr);

