# VTOL Parameter File
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
sys.path.append('../hw10')  # add parent directory
import VTOLParamHW10 as P10
# import numpy as np
# from scipy import signal
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute open-loop transfer functions
P_lon = tf(1/(P.mc+2*P.mr), [1, 0, 0])
P_lat_in = tf(1/(P.Jc+2*P.mr*P.d**2), [1, 0, 0])
P_lat_out = tf(-P.Fe/(P.mc+2*P.mr), [1, P.mu/(P.mc+2*P.mr), 0])

# Compute the controller transfer functions from HW10
C_lon = tf([(P10.kd_h+P10.kp_h*P.sigma), (P10.kp_h+P10.ki_h*P.sigma), P10.ki_h],
           [P.sigma, 1, 0])
C_lat_in = tf([(P10.kd_th+P.sigma*P10.kp_th), P10.kp_th], [P.sigma, 1])
C_lat_out = tf([(P10.kd_z+P.sigma*P10.kp_z), P10.kp_z], [P.sigma, 1])

# Plot the closed loop and open loop bode plots for the longitudinal loop
plt.figure(3), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lon*C_lon, dB=True)
cnt.bode(P_lon*C_lon/(1+P_lon*C_lon), dB = True)

# Plot the closed loop and open loop bode plots for the lateral inner loop
plt.figure(4), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lat_in*C_lat_in, dB=True)
cnt.bode(P_lat_in*C_lat_in/(1+P_lat_in*C_lat_in), dB = True)

# Plot the closed loop and open loop bode plots for the lateral outer loop
plt.figure(5), plt.clf(), plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lat_out*C_lat_out, dB=True)
cnt.bode(P_lat_out*C_lat_out/(1+P_lat_out*C_lat_out), dB = True)

# Calculate the phase and gain margin
gm, pm, Wcg, Wcp = cnt.margin(P_lon*C_lon)
print("Longitudinoal Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

gm, pm, Wcg, Wcp = cnt.margin(P_lat_in*C_lat_in)
print("Lateral Inner Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

gm, pm, Wcg, Wcp = cnt.margin(P_lat_out*C_lat_out)
print("Lateral Outer Loop: gm: ",gm," pm: ", pm," Wcg: ", Wcg, " Wcp: ", Wcp)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
