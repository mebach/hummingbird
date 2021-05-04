import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import satelliteParam as P
from hw3.satelliteDynamics import satelliteDynamics
from satelliteController import satelliteController
from hw2.signalGenerator import signalGenerator
from hw2.satelliteAnimation import satelliteAnimation
from hw2.dataPlotter import dataPlotter

# instantiate satellite, controller, and reference classes
satellite = satelliteDynamics()
controller = satelliteController()
reference = signalGenerator(amplitude=15.0*np.pi/180.0, frequency=0.04)
disturbance = signalGenerator(amplitude=1.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = satelliteAnimation()

t = P.t_start  # time starts at t_start
y = satellite.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = disturbance.step(t)  # input disturbance
        n = 0.0  #noise.random(t)  # simulate sensor noise
        x = satellite.state
        u = controller.update(r, x)  # update controller
        y = satellite.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(satellite.state)
    dataPlot.update(t, r, satellite.state, u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
