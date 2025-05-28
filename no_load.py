import numpy as np

from dc_motor import DCMotor
from signals import SquareWave
from scope import Scope
from simulation import Simulation

from parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5
t_values = np.arange(0, duration, dt)

# create motor and refernce signal
motor = DCMotor(Ra, La, J, k, b)
u_reference = SquareWave(0.35, 12, 0)

# simulation
u_values, i_values, w_values = Simulation.simulate(motor, u_reference, t_values)

# plot results
title = "MAXON A-max 32 24 V Brushless DC Motor\nNo Load Open Loop Simulation"
Scope.plot(title, t_values, u_values, i_values, w_values)
