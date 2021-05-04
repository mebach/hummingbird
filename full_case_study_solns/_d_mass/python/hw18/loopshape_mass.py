import sys
sys.path.append('..')  # add parent directory
import massParam as P
import matplotlib.pyplot as plt
from control import TransferFunction as tf
import control as cnt
import numpy as np 
sys.path.append('../hw10')  # add parent directory
import massParamHW10 as P10


def bode(*args, **kwargs):
    """
    Wraps the control library's ``bode`` function and passes all the parameters
    straight through. Returns the same thing as ``control.bode``,
    except that now it's:

    Returns
    -------
    mag : float
        Magnitude in dB
    phase : float
        Phase in degrees
    omega : np.array
        x-axis values over the range of ``mag`` and ``phase``.
    """
    mag, phase, omega = cnt.bode(*args, **kwargs)
    mag = cnt.mag2db(mag)
    phase = np.rad2deg(phase)
    return mag, phase, omega


Plant = tf([1.0/P.m],
           [1, P.b/P.m, P.k/P.m])

# PLOT = True
PLOT = False

if PLOT: 
    plt.figure(3), plt.clf() #, plt.hold(True)
mag, phase, omega = bode(Plant, dB=True, omega=np.logspace(-3, 5), Plot=False)
if PLOT: 
    plt.subplot(2, 1, 1), plt.grid(True)#, plt.yscale("log")
    plantMagPlot, = plt.semilogx(omega, mag, label='Plant')
    plt.subplot(2, 1, 2), plt.grid(True)
    plantPhasePlot, = plt.semilogx(omega, phase, label='Plant')

#########################################
#   Define Design Specifications
#########################################

#----------- noise specification --------
omega_n = 500    # attenuate noise above this frequency
gamma_n = 0.001    # attenuate noise by this amount
w = np.logspace(np.log10(omega_n), np.log10(omega_n)+2)
if PLOT:
    plt.subplot(211)
    noisePlot, = plt.plot(w, (20*np.log10(gamma_n))*np.ones(len(w)), color='g', label='noise spec')

#----------- general tracking specification --------
omega_r = 0.1    # track signals below this frequency
gamma_r = 0.03    # tracking error below this value
w = np.logspace(np.log10(omega_r) - 2, np.log10(omega_r))
if PLOT:
    plt.subplot(211)
    trackPlot, = plt.plot(w, 20*np.log10(1/gamma_r)*np.ones(len(w)), color='g', label='tracking spec')

#########################################
#   Control Design
#########################################
C = tf([1], [1])

# integral control: increase steady state tracking and dist rejection
k_I = 0.2  # frequency at which integral action ends
Integrator = tf([1.0, k_I], [1.0, 0.0])
C = C * Integrator
mag, phase, omega = bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
gm, pm, Wcg, Wcp = cnt.margin(Plant*C)
print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", cnt.mag2db(gm), " Wcg: ", Wcg)
if PLOT:
    plt.subplot(211),
    plantMagPlot, = plt.semilogx(omega, mag, label='PC')
    plt.subplot(212),
    plantPhasePlot, = plt.semilogx(omega, phase, label='PC')


# phase lead (|p|>|z|): increase PM (stability)
# low frequency gain = K*z/p
# high frequency gain = K
w_max = 7.0  # location of maximum frequency bump
phi_max = 60.0*np.pi/180.0
M = (1+np.sin(phi_max))/(1-np.sin(phi_max))  # lead ratio
z = w_max/np.sqrt(M)
p = w_max*np.sqrt(M)
Lead = tf([1.0/z, 1.0], [1.0/p, 1.0])
C = C * Lead
mag, phase, omega = bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
gm, pm, Wcg, Wcp = cnt.margin(Plant*C)
print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", cnt.mag2db(gm), " Wcg: ", Wcg)
if PLOT:
    plt.subplot(211),
    plantMagPlot, = plt.semilogx(omega, mag, label='PC')
    plt.subplot(212),
    plantPhasePlot, = plt.semilogx(omega, phase, label='PC')

# find gain to set crossover at w_max = 7 rad/s
mag, phase, omega = cnt.bode(Plant*C, dB=True, omega=[w_max], Plot=False)
# need the mag to not be in dB so, used cnt.bode
K = 1/mag[0]
C = K * C
mag, phase, omega = bode(Plant*C, dB=True, omega=np.logspace(-3, 5), Plot=False)
gm, pm, Wcg, Wcp = cnt.margin(Plant*C)
print(" pm: ", pm, " Wcp: ", Wcp, "gm: ", cnt.mag2db(gm), " Wcg: ", Wcg)
if PLOT:
    plt.subplot(211),
    plantMagPlot, = plt.semilogx(omega, mag, label='PC')
    plt.subplot(212),
    plantPhasePlot, = plt.semilogx(omega, phase, label='PC')

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
F = 1.0
# low pass filter
p = 1.0
LPF = tf(p, [1.0, p])
F = F * LPF


############################################
#  Create Plots
############################################
# Closed loop transfer function from R to Y - no prefilter
CLOSED_R_to_Y = (Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to Y - with prefilter
CLOSED_R_to_Y_with_F = (F*Plant*C/(1.0+Plant*C))
# Closed loop transfer function from R to U
CLOSED_R_to_U = (C/(1.0+Plant*C))
# Closed loop transfer function from R to U - with prefilter
CLOSED_R_to_U_with_F = (F*C/(1.0+Plant*C))

if PLOT:
    plt.figure(4), plt.clf()

    plt.subplot(311),  plt.grid(True)
    mag, phase, omega = bode(CLOSED_R_to_Y, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='b')
    mag, phase, omega = bode(CLOSED_R_to_Y_with_F, dB=True, Plot=False)
    plt.semilogx(omega, mag, color='g')
    plt.title('Close Loop Bode Plot')

    plt.subplot(312), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_Y, T)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_Y_with_F, T)
    plt.plot(T, yout, color='g')
    plt.ylabel('Step Response')

    plt.subplot(313), plt.grid(True)
    T = np.linspace(0, 2, 100)
    T, yout = cnt.step_response(CLOSED_R_to_U, T)
    plt.plot(T, yout, color='b')
    T, yout = cnt.step_response(CLOSED_R_to_U_with_F, T)
    plt.plot(T, yout, color='g')
    plt.ylabel('Control Effort')

    # Keeps the program from closing until the user presses a button.
    plt.pause(0.0001)  # not sure why this is needed for both figures to display
    print('Press key to close')
    plt.waitforbuttonpress()
    plt.close()

##############################################
#  Convert Controller to State Space Equations
##############################################
C_ss = cnt.tf2ss(C)  # convert to state space
F_ss = cnt.tf2ss(F)  # convert to state space
