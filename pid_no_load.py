import numpy as np

from dc_motor import DCMotor
from signals import SquareWave
from scope import Scope
from controllers import PIDController
from simulation import Simulation

from parameters import Ra, La, J, k, b

# simulation setup
duration = 5
dt = 1e-5
t_values = np.arange(0, duration, dt)

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
w_reference = SquareWave(0.35, 300, 0)

# controller setup
Kp = 1  # proportional gain
Ki = 0.5    # integral gain
Kd = 0.05    # derivative gai
u_min = 0.0  # minimum output limit [V]
u_max = 24.0  # maximum output limit [V]
controller = PIDController(w_reference, Kp, Ki, Kd, dt, u_min, u_max)

# simulation
results = Simulation.simulate(motor, t_values, None, None, controller)

# plot results
title = "MAXON A-max 32 24 V Brushless DC Motor\nClosed Loop P Control Simulation"
Scope.plot(title, t_values, results, None, w_reference)
