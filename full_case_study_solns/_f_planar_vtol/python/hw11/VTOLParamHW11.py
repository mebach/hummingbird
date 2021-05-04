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
# tuning parameters
tr_h    = 2.0
wn_h    = 2.2/tr_h
zeta_h  = 0.707

tr_z    = 2.0
wn_z    = 2.2/tr_z
zeta_z  = 0.707

M = 10.0
tr_th   = tr_z/M
wn_th   = 2.2/tr_th
zeta_th = 0.707

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

# gain calculation
des_char_poly_lon = [1.0, 2.0*zeta_h*wn_h, wn_h**2]
des_poles_lon = np.roots(des_char_poly_lon)

des_char_poly_lat = np.convolve([1.0, 2.0*zeta_z*wn_z, wn_z**2],
                                [1.0, 2.0*zeta_th*wn_th, wn_th**2])
des_poles_lat = np.roots(des_char_poly_lat)


# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A_lon, B_lon)) != 2:
    print("The longitudinal system is not controllable")
else:
    K_lon = cnt.acker(A_lon, B_lon, des_poles_lon)
    kr_lon = -1.0/(C_lon @ np.linalg.inv(A_lon-B_lon @ K_lon) @ B_lon)

if np.linalg.matrix_rank(cnt.ctrb(A_lat, B_lat)) != 4:
    print("The lateral system is not controllable")
else:
    K_lat = cnt.acker(A_lat, B_lat, des_poles_lat)
    Cr = np.array([[1.0, 0.0, 0.0, 0.0]])
    kr_lat = -1.0/(Cr @ np.linalg.inv(A_lat-B_lat @ K_lat) @ B_lat)

print('K_lon: ', K_lon)
print('kr_lon: ', kr_lon)
print('K_lat: ', K_lat)
print('kr_lat: ', kr_lat)




