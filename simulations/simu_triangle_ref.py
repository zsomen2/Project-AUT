from aut_project.dc_motor import DCMotor
from aut_project.signals import TriangleWave
from aut_project.simulation import Simulation
from aut_project.scope import Scope

from aut_project.parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
u_reference = TriangleWave(0.7, 12, 0)

# simulation
results = Simulation.simulate("open", motor, duration, dt, u_reference, None)

# plot results
title = "MAXON A-max 32 24 V DC Motor\nNo Load Open Loop Simulation\nTriangle Wave Voltage Reference"
Scope.plot(title, results)
