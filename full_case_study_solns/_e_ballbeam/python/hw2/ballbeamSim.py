import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from dataPlotter import dataPlotter


# instantiate reference input classes
reference = signalGenerator(amplitude=0.5, frequency=0.1)
zRef = signalGenerator(amplitude=0.25, frequency=0.1)
thetaRef = signalGenerator(amplitude=2.0*np.pi/4.0, frequency=0.1)
fRef = signalGenerator(amplitude=5, frequency=.5)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = ballbeamAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # set variables
    r = reference.square(t)
    z = zRef.sin(t)
    theta = thetaRef.sin(t)
    f = fRef.sawtooth(t)
    # update animation
    state = np.array([[0.25+z], [theta], [0.0], [0.0]])
    animation.update(state)
    dataPlot.update(t, r, state, f)

    t = t + P.t_plot  # advance time by t_plot
    plt.pause(0.1)

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
