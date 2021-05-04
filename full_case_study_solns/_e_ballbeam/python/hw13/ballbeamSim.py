import sys
sys.path.append('..')  # add parent directory
import numpy as np
import matplotlib.pyplot as plt
import ballbeamParam as P
from hw3.ballbeamDynamics import ballbeamDynamics
from ballbeamController import ballbeamController
from hw2.signalGenerator import signalGenerator
from hw2.ballbeamAnimation import ballbeamAnimation
from hw2.dataPlotter import dataPlotter
from dataPlotterObserver import dataPlotterObserver

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics(alpha=0.0)
controller = ballbeamController()
reference = signalGenerator(amplitude=0.125, frequency=0.04, y_offset=0.25)
disturbance = signalGenerator(amplitude=1.0)
noise_z = signalGenerator(amplitude=0.01)
noise_th = signalGenerator(amplitude=0.01)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
dataPlotObserver = dataPlotterObserver()
animation = ballbeamAnimation()

t = P.t_start  # time starts at t_start
y = ballbeam.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        r = reference.square(t)  # reference input
        d = 0 #disturbance.step(t)  # input disturbance
        n = np.array([[0.0], [0.0]]) #np.array([[noise_z.random(t)], [noise_th.random(t)]])  # simulate sensor noise
        u, xhat = controller.update(r, y + n)  # update controller
        y = ballbeam.update(u + d)  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(ballbeam.state)
    dataPlot.update(t, r, ballbeam.state, u)
    dataPlotObserver.update(t, ballbeam.state, xhat, d, 0.0)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
