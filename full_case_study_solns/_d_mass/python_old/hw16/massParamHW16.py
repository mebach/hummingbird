# Inverted Pendulum Parameter File
import sys
sys.path.append('..')  # add parent directory
import massParam as P
sys.path.append('../hw10')  # add parent directory
import massParamHW10 as P10
import numpy as np
# from scipy import signal
import control as cnt
from control import TransferFunction as tf
import matplotlib.pyplot as plt

# Compute plant transfer functions
Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])

# Compute transfer function of controller
C_pid = tf([(P10.kd+P10.kp*P.sigma), (P10.kp+P10.ki*P.sigma), P10.ki],
           [P.sigma, 1, 0])


# display bode plots of transfer functions
plt.figure(3), plt.clf, plt.hold(True), plt.grid(True)
cnt.matlab.bode(Plant, Plant*C_pid, dB=True)
#plt.legend('No control', 'PID')
plt.title('Mass Spring Damper')


# Closes plot windows when the user presses a button.
plt.pause(0.0001)  # not sure why this is needed for both figures to display
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
