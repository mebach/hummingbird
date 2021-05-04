# ballbeam Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import numpy as np
from scipy import signal
import control as cnt

####################################################
#                 State Space
####################################################
# tuning parameters
tr_z = 1.2        # rise time for position
tr_theta = 0.5    # rise time for angle
zeta_z = 0.707  # damping ratio position
zeta_th = 0.707  # damping ratio angle
integrator_pole = -5.0
# pick observer poles
wn_th_obs = 8.0*2.2/tr_theta
wn_z_obs = 5.0*2.2/tr_z
dist_obsv_pole = -10.0

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.matrix([[0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0],
               [0.0, -P.m1*P.g/((P.m2*P.length**2)/3.0+P.m1*(P.length/2.0)**2), 0.0, 0.0],
               [-P.g, 0.0, 0.0, 0.0]])

B = np.matrix([[0.0],
               [0.0],
               [P.length/(P.m2*P.length**2/3.0+P.m1*P.length**2/4.0)],
               [0.0]])

C = np.matrix([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0]])

# form augmented system
A1 = np.matrix([[0.0, 0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 0.0, 1.0, 0.0],
               [0.0, -P.m1*P.g/((P.m2*P.length**2)/3.0+P.m1*(P.length/2.0)**2), 0.0, 0.0, 0.0],
               [-P.g, 0.0, 0.0, 0.0, 0.0],
               [0.0, -1.0, 0.0, 0.0, 0.0]])

B1 = np.matrix([[0.0],
               [0.0],
               [P.length/(P.m2*P.length**2/3.0+P.m1*P.length**2/4.0)],
               [0.0],
               [0.0]])

# gain calculation
wn_th = 2.2/tr_theta  # natural frequency for angle
wn_z = 2.2/tr_z  # natural frequency for position
des_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z, wn_z**2],
                [1, 2*zeta_th*wn_th, wn_th**2]),
    np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 5:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.matrix([K1.item(0), K1.item(1), K1.item(2), K1.item(3)])
    ki = K1.item(4)

# compute observer gains
# Augmented Matrices
A2 = np.concatenate((
        np.concatenate((A, B), axis=1),
        np.zeros((1, 5))),
        axis=0)
C2 = np.concatenate((C, np.zeros((2, 1))), axis=1)

des_obs_char_poly = np.convolve(
    np.convolve([1, 2*zeta_z*wn_z_obs, wn_z_obs**2], [1, 2*zeta_th*wn_th_obs, wn_th_obs**2]),
    np.poly(dist_obsv_pole))
des_obs_poles = np.roots(des_obs_char_poly)

# Compute the gains if the system is observable
if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 5:
    print("The system is not observable")
else:
    # place_poles returns an object with various properties.  The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L2 = signal.place_poles(A2.T, C2.T, des_obs_poles).gain_matrix.T
    L = L2[0:4, 0:2]
    Ld = L2[4:5, 0:2]


print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)
print('Ld: ', Ld)
