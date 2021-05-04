import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np 

P_lat_in = tf(1/(P.Jc+2*P.mr*P.d**2), [1, 0, 0])

Plant = P_lat_in

#PLOT = True
PLOT = False

if PLOT: plt.figure(5), plt.clf() #, plt.hold(True)
mag, phase, omega = cnt.bode(Plant, dB=True, omega=np.logspace(-3, 5), Plot=False)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)
    plantMagPlot, = plt.semilogx(omega, mag, label='Plant')
    plt.subplot(2, 1, 2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#########################################


#########################################
#   Control Design
#########################################

C = tf([1], [1])

#  phase lead: increase PM (stability)
w_max = 8.0 #location of maximum frequency bump (desired crossover)
M = 15.0  # lead ratio
Lead = tf([M, M*w_max/np.sqrt(M)], [1, w_max*np.sqrt(M)])
C = C*Lead
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)


#  low pass filter: decrease gain at high frequency (noise)
p = 200.0
LPF = tf(p, [1, p])
C = C*LPF
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C, dB=True, Plot=False)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)


############################################
#  Create Plots
############################################
# Open-loop transfer function
OPEN = Plant*C
# Closed loop transfer function from R to Y
CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1.0+Plant*C))

mag, phase, omega = cnt.bode(OPEN, omega=np.logspace(-3, 5), dB=True, Plot=False)
if PLOT: 
    openMagPlot, = plt.semilogx(omega, mag, label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, label='OPEN')
    plt.legend(handles=[plantMagPlot, openMagPlot])
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

if PLOT:
    plt.figure(6), plt.clf()

    plt.subplot(311),  plt.grid(True)
    mag, phase, omega = cnt.bode(CLOSED_R_to_Y, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    plt.title('Close Loop Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    plt.title('Control Effort for Step Response')

if PLOT:
    # Keeps the program from closing until the user presses a button.
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = cnt.tf2ss(C)  # convert to state space

# ########################################################################
# #  Convert Controller to discrete transfer functions for implementation
# ########################################################################
# C_in_d = tf.sample(C, P.Ts, method='bilinear') #bilinear: Tustin's approximation ("generalized bilinear transformation" with alpha=0.5)

