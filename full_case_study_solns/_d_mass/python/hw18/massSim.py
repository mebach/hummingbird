import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from hw3.massDynamics import massDynamics
from massController import massController
from hw2.signalGenerator import signalGenerator
from hw2.massAnimation import massAnimation
from hw2.dataPlotter import dataPlotter


# instantiate satellite, controller, and reference classes
mass = massDynamics()
controller = massController()
reference = signalGenerator(amplitude=0.5, frequency=0.04)
disturbance = signalGenerator(amplitude=0.25)
noise = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = massAnimation()


t = P.t_start  # time starts at t_start
y = mass.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = noise.random(t)  # simulate sensor noise
        u = controller.update(r, y + n)  # update controller
        y = mass.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, r, mass.state, u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
