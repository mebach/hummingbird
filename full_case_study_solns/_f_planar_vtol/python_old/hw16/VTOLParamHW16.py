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

# display bode plots of transfer functions
plt.figure(3), plt.clf, plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lon, P_lon*C_lon, dB=True)
plt.title('VTOL Longitudinal Loop')

# display bode plots of transfer functions
plt.figure(4), plt.clf, plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lat_in, P_lat_in*C_lat_in, dB=True)
#plt.legend('No control', 'PD')
plt.title('VTOL Lateral Inner Loop')

plt.figure(5), plt.clf, plt.hold(True), plt.grid(True)
cnt.matlab.bode(P_lat_out, P_lat_out*C_lat_out, tf([1.0], [1.0, 0.0]), dB=True)
#legend('No control', 'PID','1/s')
plt.title('VTOL Lateral Outer Loop')


# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
