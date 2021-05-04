# VTOL Parameter File
import numpy as np
from scipy import signal
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P


####################################################
#                 State Space
####################################################
# tuning parameters
wn_h    = 1.0
zeta_h  = 0.707
wn_z    = 0.9905
zeta_z  = 0.707
wn_th   = 13.3803
zeta_th = 0.707
integrator_h = [-3.0]
integrator_z = [-4.0]

# observer gains
wn_h_obs    = 10.0*wn_h
wn_z_obs    = 10.0*wn_z
wn_th_obs   = 5.0*wn_th


# State Space Equations
A_lon = np.array([[0.0, 1.0],
                   [0.0, 0.0]])
B_lon = np.array([[0.0],
                   [1.0/(P.mc+2.0*P.mr)]])
C_lon = np.array([[1.0, 0.0]])
A_lat = np.array([[0.0, 0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0],
                   [0.0, -P.Fe/(P.mc+2.0*P.mr), -(P.mu/(P.mc+2.0*P.mr)), 0.0],
                   [0.0, 0.0, 0.0, 0.0]])
B_lat = np.array([[0.0],
                   [0.0],
                   [0.0],
                   [1.0/(P.Jc+2*P.mr*P.d**2)]])
C_lat = np.array([[1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, 0.0]])

# form augmented system
A1_lon = np.array([[0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0]])
B1_lon = np.array([[0.0],
                    [1.0],
                    [0.0]])
A1_lat = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, -P.Fe/(P.mc+2.0*P.mr), -(P.mu/(P.mc+2.0*P.mr)), 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 0.0, 0.0]])
B1_lat = np.array([[0.0],
                    [0.0],
                    [0.0],
                    [1.0/(P.Jc+2*P.mr*P.d**2)],
                    [0.0]])

# gain calculation
des_char_poly_lon = np.convolve([1.0, 2.0*zeta_h*wn_h, wn_h**2],
                                np.poly(integrator_h))
des_poles_lon = np.roots(des_char_poly_lon)

des_char_poly_lat = np.convolve(
                        np.convolve([1.0, 2.0*zeta_z*wn_z, wn_z**2],
                                    [1.0, 2.0*zeta_th*wn_th, wn_th**2]),
                        np.poly(integrator_z))
des_poles_lat = np.roots(des_char_poly_lat)


# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1_lon, B1_lon)) != 3:
    print("The longitudinal system is not controllable")
else:
    K1_lon = cnt.acker(A1_lon, B1_lon, des_poles_lon)
    K_lon = np.matrix([K1_lon.item(0), K1_lon.item(1)])
    ki_lon = K1_lon.item(2)

if np.linalg.matrix_rank(cnt.ctrb(A1_lat, B1_lat)) != 5:
    print("The lateral system is not controllable")
else:
    K1_lat = cnt.acker(A1_lat, B1_lat, des_poles_lat)
    K_lat = np.matrix([K1_lat.item(0), K1_lat.item(1), K1_lat.item(2), K1_lat.item(3)])
    ki_lat = K1_lat.item(4)

# compute observer poles
des_obs_poles_lon = np.roots([1.0, 2.0*zeta_h*wn_h_obs, wn_h_obs**2])
des_char_poly_lat = np.convolve([1.0, 2.0*zeta_z*wn_z_obs, wn_z_obs**2],
                                [1.0, 2.0*zeta_th*wn_th_obs, wn_th_obs**2])
des_obs_poles_lat = np.roots(des_char_poly_lat)

if np.linalg.matrix_rank(cnt.ctrb(A_lon.T, C_lon.T)) != 2:
    print("The longitudinal system is not observable")
else:
    # place_poles returns an object with various properties.  The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L_lon = signal.place_poles(A_lon.T, C_lon.T, des_obs_poles_lon).gain_matrix.T

if np.linalg.matrix_rank(cnt.ctrb(A_lat.T, C_lat.T)) != 4:
    print("The lateral system is not observable")
else:
    # place_poles returns an object with various properties.  The gains are accessed through .gain_matrix
    # .T transposes the matrix
    L_lat = signal.place_poles(A_lat.T, C_lat.T, des_obs_poles_lat).gain_matrix.T

print('K_lon: ', K_lon)
print('ki_lon: ', ki_lon)
print('L_lon^T: ', L_lon.T)
print('K_lat: ', K_lat)
print('ki_lat: ', ki_lat)
print('L_lat^T: ', L_lat.T)




