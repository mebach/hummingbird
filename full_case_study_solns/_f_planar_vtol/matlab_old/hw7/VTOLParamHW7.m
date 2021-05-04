% inverted ballbeam - parameter file for hw8
addpath ./.. % adds the parent directory to the path
VTOLParam % general ballbeam parameters

% select desired closed loop char eq
Delta_cl_d = poly([-.3,-.2]);

% PD gains
P.kp_h = Delta_cl_d(3)*(P.mc+2*P.mr);
P.kd_h = Delta_cl_d(2)*(P.mc+2*P.mr);

fprintf('\t kp_z: %f\n', P.kp_h)
fprintf('\t kd_z: %f\n', P.kd_h)
