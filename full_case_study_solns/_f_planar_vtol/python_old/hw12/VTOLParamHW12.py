# VTOL Parameter File
import numpy as np
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
integrator_h = -1.0
integrator_z = -1.0

# State Space Equations
A_lon = np.matrix([[0.0, 1.0],
                   [0.0, 0.0]])
B_lon = np.matrix([[0.0],
                   [1.0/(P.mc+2.0*P.mr)]])
C_lon = np.matrix([[1.0, 0.0]])
A_lat = np.matrix([[0.0, 0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0],
                   [0.0, -P.Fe/(P.mc+2.0*P.mr), -(P.mu/(P.mc+2.0*P.mr)), 0.0],
                   [0.0, 0.0, 0.0, 0.0]])
B_lat = np.matrix([[0.0],
                   [0.0],
                   [0.0],
                   [1.0/(P.Jc+2*P.mr*P.d**2)]])
C_lat = np.matrix([[1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, 0.0]])

# form augmented system
A1_lon = np.matrix([[0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0]])
B1_lon = np.matrix([[0.0],
                    [1.0],
                    [0.0]])
A1_lat = np.matrix([[0.0, 0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0, 0.0],
                    [0.0, -P.Fe/(P.mc+2.0*P.mr), -(P.mu/(P.mc+2.0*P.mr)), 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0, 0.0],
                    [-1.0, 0.0, 0.0, 0.0, 0.0]])
B1_lat = np.matrix([[0.0],
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

print('K_lon: ', K_lon)
print('ki_lon: ', ki_lon)
print('K_lat: ', K_lat)
print('ki_lat: ', ki_lat)




