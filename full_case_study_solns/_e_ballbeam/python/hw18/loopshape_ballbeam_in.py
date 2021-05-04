import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np 
sys.path.append('../hw10')  # add parent directory
import ballbeamParamHW10 as P10

P_in = tf([P10.b0],[1,0,0])

Plant = P_in

# PLOT = True
PLOT = False

if PLOT: 
    plt.figure(3), plt.clf() #, plt.hold(True)
mag, phase, omega = cnt.bode(Plant, dB=True, omega=np.logspace(-3, 5), Plot=False)
mag = cnt.mag2db(mag)
phase = np.rad2deg(phase)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)#, plt.yscale("log")
    plantMagPlot, = plt.semilogx(omega, mag, label='Plant')
    plt.subplot(2, 1, 2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#########################################

#----------- general tracking specification --------
omega_r = 10**(0.0)  # track signals below this frequency
gamma_r = 10.0**(-50.0/20.0)  # tracking error below this value
w = np.logspace(np.log10(omega_r)-2, np.log10(omega_r))
if PLOT:
    plt.subplot(211)
    trackPlot, = plt.plot(w, (20*np.log10(1.0/gamma_r))*np.ones(len(w)), color='g', label='tracking spec')

#----------- noise specification --------
omega_n = 10**3    # attenuate noise above this frequency
gamma_n = 10**(-50.0/20.0)    # attenuate noise by this amount
w = np.logspace(np.log10(omega_n), np.log10(omega_n)+2)
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w, (20*np.log10(gamma_n))*np.ones(len(w)), color='g', label='noise spec')


#########################################
#   Control Design
#########################################

C = tf([1], [1])

# Proportional control: change cross over frequency
K = tf([150.0], [1])
C = C*K
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C , dB=True, omega=np.logspace(-3, 5), Plot=False)
    mag = cnt.mag2db(mag)
    phase = np.rad2deg(phase)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)

#  phase lead: increase PM (stability)
w_max = 40.0 #location of maximum frequency bump (desired crossover)
M = 15.0  # lead ratio
Lead = tf([M, M*w_max/np.sqrt(M)], [1, w_max*np.sqrt(M)])
C = C*Lead
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
    mag = cnt.mag2db(mag)
    phase = np.rad2deg(phase)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)

# find gain to set crossover at w_max = 25 rad/s
mag, phase, omega = cnt.bode(Plant*C, dB=False, omega=[w_max], Plot=False)
K = tf([1/mag.item(0)], [1])
C = K*C
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
    mag = cnt.mag2db(mag)
    phase = np.rad2deg(phase)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)

#  -----low pass filter: decrease gain at high frequency (noise)----
p = 500.0
LPF = tf([p], [1, p])
C = C*LPF
if PLOT:
    mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
    mag = cnt.mag2db(mag)
    phase = np.rad2deg(phase)
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega, mag, color='r', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega, phase, color='r', label='OPEN')
    # Calculate the phase and gain margin
    gm, pm, Wcg, Wcp = cnt.margin(Plant * C)
    print("pm: ", pm, " gm: ", gm, " Wcp: ", Wcp, " Wcg: ", Wcg)


# ###########################################
#  Create Plots
# ###########################################
# Open-loop transfer function
OPEN = Plant*C
# Closed loop transfer function from R to Y
CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1.0+Plant*C))

mag, phase, omega = cnt.bode(OPEN, omega=np.logspace(-3, 5), dB=True, Plot=False)
mag = cnt.mag2db(mag)
phase = np.rad2deg(phase)
if PLOT: 
    plt.subplot(211)
    openMagPlot, = plt.semilogx(omega,mag, color='k', label='OPEN')
    plt.subplot(212)
    openPhasePlot, = plt.semilogx(omega,phase, color='k', label='OPEN')
    plt.legend(handles=[plantMagPlot, noisePlot, openMagPlot])
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

if PLOT:
    plt.figure(4), plt.clf()

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

########################################################################
#  Convert Controller to discrete transfer functions for implementation
########################################################################
C_in_d = tf.sample(C, P.Ts, method='bilinear') #bilinear: Tustin's approximation ("generalized bilinear transformation" with alpha=0.5)

