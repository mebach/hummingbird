# Single link arm Parameter File
import sys
sys.path.append('..')  # add parent directory
import mass Param as P
import numpy as np
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt


# Compute plant transfer functions
Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])


# Bode plot of the plant
plt.figure(3), cnt.bode_plot(Plant, dB=True)

# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
