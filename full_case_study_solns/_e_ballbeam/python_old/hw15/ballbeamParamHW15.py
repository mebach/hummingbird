# Ball on Beam Parameter File
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute inner and outer open-loop transfer functions
P_in = tf([P.length/(P.m2*P.length**2/3.0+P.m1*(P.length/2.0)**2)],
          [1, 0, 0])
P_out = tf(-P.g, [1, 0, 0])

# Plot the closed loop and open loop bode plots for the inner loop
plt.figure(3), cnt.bode_plot(P_in, dB=True)
plt.figure(4), cnt.bode_plot(P_out, dB=True)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
