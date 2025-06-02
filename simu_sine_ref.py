import numpy as np

from dc_motor import DCMotor
from signals import SineWave
from simulation import Simulation
from scope import Scope

from parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5
t_values = np.arange(0, duration, dt)

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
u_reference = SineWave(0.7, 6, 6)

# simulation
results = Simulation.simulate(motor, t_values, None, u_reference)

# plot results
title = "MAXON A-max 32 24 V DC Motor\nNo Load Open Loop Simulation\nSine Wave Voltage Reference"
Scope.plot(title, t_values, results)
