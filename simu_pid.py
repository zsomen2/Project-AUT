import numpy as np

from dc_motor import DCMotor
from signals import SquareWave
from controllers import PIDController
from simulation import Simulation
from scope import Scope

from parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5
t_values = np.arange(0, duration, dt)

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
w_reference = SquareWave(0.35, 300, 0)

# controller setup
Kp = 1       # proportional gain
Ki = 0.5     # integral gain
Kd = 0.05    # derivative gai
u_min = 0.0  # minimum output limit [V]
u_max = 24.0 # maximum output limit [V]
controller = PIDController(w_reference, Kp, Ki, Kd, dt, u_min, u_max)

# simulation
results = Simulation.simulate_closed_loop(motor, t_values, controller, None)

# plot results
title = "MAXON A-max 32 24 V DC Motor\nClosed Loop PID Control Simulation"
Scope.plot(title, t_values, results, w_reference)
