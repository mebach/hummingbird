# mass Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import massParam as P

# PD gains
#kp = 4.5
#kd = 12.0

# open loop coefficients
a1 = P.b/P.m
a0 = P.k/P.m

# select desired closed loop char eq
Delta_cl_d = np.poly([-1,-1.5])

# PD gains
kp = P.m*(Delta_cl_d[2]-a0)
kd = P.m*(Delta_cl_d[1]-a1)


print('kp: ', kp)
print('kd: ', kd)



