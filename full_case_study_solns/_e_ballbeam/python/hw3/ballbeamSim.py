import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import ballbeamParam as P
from hw2.signalGenerator import signalGenerator
from hw2.ballbeamAnimation import ballbeamAnimation
from hw2.dataPlotter import dataPlotter
from ballbeamDynamics import ballbeamDynamics

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()
reference = signalGenerator(amplitude=0.5, frequency=0.02)
force = signalGenerator(amplitude=.001, frequency=1)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = ballbeamAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)
        u = force.sin(t)
        y = ballbeam.update(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(ballbeam.state)
    dataPlot.update(t, r, ballbeam.state, u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
