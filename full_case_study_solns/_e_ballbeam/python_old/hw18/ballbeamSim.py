import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import ballbeamParam as P
from ballbeamDynamics import ballbeamDynamics
from ballbeamController import ballbeamController
from signalGenerator import signalGenerator
from ballbeamAnimation import ballbeamAnimation
from plotData import plotData

# instantiate ballbeam, controller, and reference classes
ballbeam = ballbeamDynamics()
ctrl = ballbeamController()
reference = signalGenerator(amplitude=0.125, frequency=0.04)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = ballbeamAnimation()

# set disturbance input
disturbance = 1.0

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    ref_input[0] += 0.25
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot: # updates control and dynamics at faster simulation rate
        u = ctrl.u(ref_input, ballbeam.outputs())  # Calculate the control value
        input = [u[0]+disturbance]
        ballbeam.propagateDynamics(input)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.drawBallbeam(ballbeam.states())
    dataPlot.updatePlots(t, ref_input, ballbeam.states(), u)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
