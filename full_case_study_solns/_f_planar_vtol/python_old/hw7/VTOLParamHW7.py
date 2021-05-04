# VTOL Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P

# select desired closed loop char eq
Delta_cl_d = np.poly([-.3,-.2])

# PD gains
kp_h = Delta_cl_d[2]*(P.mc+2.0*P.mr)
kd_h = Delta_cl_d[1]*(P.mc+2.0*P.mr)


print('kp_h: ', kp_h)
print('kd_h: ', kd_h)



