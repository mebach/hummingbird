import matplotlib.pyplot as plt
import sys
sys.path.append('..')  # add parent directory
import massParam as P
from hw_a.signalGenerator import signalGenerator
from hw_a.massAnimation import massAnimation
from hw_a.plotData import plotData
from massDynamics import massDynamics

# instantiate mass, controller, and reference classes
mass = massDynamics()
reference = signalGenerator(amplitude=0.01, frequency=0.02)
force = signalGenerator(amplitude=1.0, frequency=0.05)

# instantiate the simulation plots and animation
dataPlot = plotData()
animation = massAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Get referenced inputs from signal generators
    ref_input = reference.square(t)
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        f=force.square(t)
        mass.update(f)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(mass.state)
    dataPlot.update(t, ref_input, mass.state, f)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
