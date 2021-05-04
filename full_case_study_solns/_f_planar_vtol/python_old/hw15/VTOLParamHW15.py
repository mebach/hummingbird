# VTOL Parameter File
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute open-loop transfer functions
P_lon = tf(1/(P.mc+2*P.mr), [1, 0, 0])
P_lat_in = tf(1/(P.Jc+2*P.mr*P.d**2), [1, 0, 0])
P_lat_out = tf(-P.Fe/(P.mc+2*P.mr), [1, P.mu/(P.mc+2*P.mr), 0])

# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(3), cnt.bode_plot(P_lon, dB=True)
plt.figure(4), cnt.bode_plot(P_lat_in, dB=True)
plt.figure(5), cnt.bode_plot(P_lat_out, dB=True)


# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
