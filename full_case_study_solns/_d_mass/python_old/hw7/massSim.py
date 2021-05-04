import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import numpy as np
import massParam as P
from massDynamics import massDynamics
from massController import massController
from signalGenerator import signalGenerator
from massAnimation import massAnimation
from plotData import plotData

# instantiate mass, controller, and reference classes
mass = massDynamics()
ctrl = massController()
reference = signalGenerator(amplitude=0.5, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = massAnimation()

# set disturbance input
disturbance = 0.25


t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, mass.outputs())  # Calculate the control value
        sys_input = [u[0]+disturbance]  # input to  plant is control input + disturbance
        mass.propagateDynamics(sys_input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawMass(mass.states())
    dataPlot.updatePlots(t, ref_input, mass.states(), u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
