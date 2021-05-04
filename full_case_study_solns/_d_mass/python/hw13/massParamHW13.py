# Single link mass Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import massParam as P

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
F_max = P.F_max  # limit on control signal

#  tuning parameters
tr = 2.5
zeta = 0.707
integrator_pole = [-10.0]
tr_obs = tr/10.0  # rise time for observer
zeta_obs = 0.707  # damping ratio for observer

# State Space Equations
# xdot = A*x + B*u
# y = C*x
A = np.array([[0.0, 1.0],
               [-P.k/P.m, -P.b/P.m]])

B = np.array([[0.0],
               [1.0/P.m]])

C = np.array([[1.0, 0.0]])

# form augmented system
A1 = np.array([[0.0, 1.0, 0.0],
               [-P.k/P.m, -P.b/P.m, 0.0],
               [-1.0, 0.0, 0.0]])

B1 = np.array([[0.0],
               [1.0/P.m],
               [0.0]])

# gain calculation
wn = 2.2/tr  # natural frequency
des_char_poly = np.convolve(
    [1, 2*zeta*wn, wn**2],
    np.poly(integrator_pole))
des_poles = np.roots(des_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.array([K1.item(0), K1.item(1)])
    ki = K1.item(2)

# observer design
wn_obs = 2.2/tr_obs
des_obsv_char_poly = [1, 2*zeta_obs*wn_obs, wn_obs**2]
des_obsv_poles = np.roots(des_obsv_char_poly)

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A.T, C.T)) != 2:
    print("The system is not observerable")
else:
    L = cnt.acker(A.T, C.T, des_obsv_poles).T

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)



