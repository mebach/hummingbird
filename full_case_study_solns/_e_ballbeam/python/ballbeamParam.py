# Ball on Beam Parameter File
import numpy as np
# import control as cnt

# Physical parameters of the  ballbeam known to the controller
m1 = 0.35  # Mass of the ball, kg
m2 = 2  # mass of beam, kg
length = 0.5  # length of beam, m
g = 9.81  # gravity at sea level, m/s^2

# parameters for animation
radius = 0.05  # radius of ball

# Initial Conditions
z0 = length/2.0  # initial ball position,m
theta0 = 0.0*np.pi/180  # initial beam angle,rads
zdot0 = 0.0  # initial speed of ball along beam, m/s
thetadot0 = 0.0  # initial angular speed of theh beam,rads/s

# Simulation Parameters
t_start = 0.0  # Start time of simulation
t_end = 50.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1  # the plotting and animation is updated at this rate

# saturation limits
Fmax = 15.0  # Max Force, N

# dirty derivative parameters
sigma = 0.05  # cutoff freq for dirty derivative
beta = (2.0*sigma-Ts)/(2.0*sigma+Ts)  # dirty derivative gain

# equilibrium force when ball is in center of beam
ze = length/2.0
Fe = m1*g*z0/length + m2*g/2.0
