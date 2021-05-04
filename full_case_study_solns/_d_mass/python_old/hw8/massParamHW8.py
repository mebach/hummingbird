# Single link arm Parameter File
import numpy as np
# import control as cnt
import sys
sys.path.append('..')  # add parent directory
import massParam as P

Ts = P.Ts  # sample rate of the controller
beta = P.beta  # dirty derivative gain
tau_max = P.F_max  # limit on control signal

#  tuning parameters
tr = 2.2
zeta = 0.707


# compute PD gains
# open loop char polynomial and poles
a1 = P.b/P.m
a0 = P.k/P.m

wn = 2.2/tr
alpha1 = 2.0*zeta*wn
alpha0 = wn**2

kp = P.m*(alpha0-a0)
kd = P.m*(alpha1-a1)

print('kp: ', kp)
print('kd: ', kd)



