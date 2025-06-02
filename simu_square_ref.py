import numpy as np

from dc_motor import DCMotor
from signals import SquareWave
from simulation import Simulation
from scope import Scope

from parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
u_reference = SquareWave(0.35, 12, 0, 0.5)

# simulation
results = Simulation.simulate("open", motor, duration, dt, u_reference, None)

# plot results
title = "MAXON A-max 32 24 V DC Motor\nNo Load Open Loop Simulation\nSquare Wave Voltage Reference"
Scope.plot(title, results)
