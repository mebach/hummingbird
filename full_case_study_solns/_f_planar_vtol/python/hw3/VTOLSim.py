import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
from hw2.signalGenerator import signalGenerator
from hw2.VTOLAnimation import VTOLAnimation
from hw2.dataPlotter import dataPlotter
from VTOLDynamics import VTOLDynamics

# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
z_reference = signalGenerator(amplitude=0.5, frequency=0.02)
h_reference = signalGenerator(amplitude=0.5, frequency=0.02)
force = signalGenerator(amplitude=0.001, frequency=1.0)
torque = signalGenerator(amplitude=0.001, frequency=1.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        z_ref = z_reference.square(t)
        h_ref = h_reference.sin(t)
        f = (P.mc+2.0*P.mr)*P.g #P.Fe + force.sin(t)
        tau = 0.0 # torque.sin(t)
        u = P.mixing @ np.array([[f], [tau]])
        VTOL.update(u)  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(VTOL.state, z_ref)
    dataPlot.update(t, VTOL.state, z_ref, h_ref, f, tau)
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
