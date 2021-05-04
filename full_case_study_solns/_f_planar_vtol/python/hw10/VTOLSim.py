import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append('..')  # add parent directory
import VTOLParam as P
from hw3.VTOLDynamics import VTOLDynamics
from VTOLController import VTOLController
from hw2.signalGenerator import signalGenerator
from hw2.VTOLAnimation import VTOLAnimation
from hw2.dataPlotter import dataPlotter

# instantiate VTOL, controller, and reference classes
VTOL = VTOLDynamics()
controller = VTOLController()
z_reference = signalGenerator(amplitude=4.0, frequency=0.02, y_offset=5.0)
h_reference = signalGenerator(amplitude=3.0, frequency=0.03, y_offset=5.0)

# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()

t = P.t_start  # time starts at t_start
y = VTOL.h()  # output of system at start of simulation
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    while t < t_next_plot:  # updates control and dynamics at faster simulation rate
        h_ref = h_reference.square(t)
        z_ref = z_reference.square(t)
        r = np.array([[z_ref], [h_ref]])  # reference input
        d = np.array([[0.0], [0.0]])  #disturbance.step(t)  # input disturbance
        n = np.array([[0.0], [0.0], [0.0]])  #noise.random(t)  # simulate sensor noise
        u = controller.update(r, y + n)  # update controller
        y = VTOL.update(P.mixing @ (u + d))  # propagate system
        t = t + P.Ts  # advance time by Ts
    # update animation and data plots
    animation.update(VTOL.state, z_ref)
    dataPlot.update(t, VTOL.state, z_ref, h_ref, u.item(0), u.item(1))
    plt.pause(0.0001)  # the pause causes the figure to be displayed during the simulation

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
