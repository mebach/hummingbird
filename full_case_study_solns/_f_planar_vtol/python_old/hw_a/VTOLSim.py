import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from plotData import plotData


# instantiate reference input classes
z_reference = signalGenerator(amplitude=0.5, frequency=0.1)
h_reference = signalGenerator(amplitude=0.5, frequency=0.1)
zRef = signalGenerator(amplitude=0.5, frequency=0.1)
hRef = signalGenerator(amplitude=0.5, frequency=0.1)
thetaRef = signalGenerator(amplitude=2.0*np.pi, frequency=0.1)
fRef = signalGenerator(amplitude=5, frequency=0.5)
tauRef = signalGenerator(amplitude=5, frequency=0.5)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    z_r = z_reference.sin(t)[0]
    z = zRef.sin(t)
    h_r = h_reference.square(t)[0]
    h = zRef.sin(t)
    theta = thetaRef.sin(t)
    f = fRef.sawtooth(t)[0]
    tau = fRef.sawtooth(t)[0]
    # update animation
    state = [z[0], h[0], theta[0], 0.0, 0.0, 0.0]
    animation.drawVTOL(state, z_r)
    #dataPlot.updatePlots(t, state, z_r, h_r, f, tau)

    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
