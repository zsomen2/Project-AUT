import numpy as np
from scipy.integrate import odeint

from dc_motor import DCMotor
from signals import SquareWave
from scope import Scope

from parameters import Ra, La, J, k, b

# simulation setup
duration = 10
dt = 1e-5
t_values = np.arange(0, duration, dt)

# create motor and refernce signal
motor = DCMotor(Ra, La, J, k, b)
u = SquareWave(freq=1, u_high=12, u_low=0)

def simulate(dc_motor, u, t_values, x0=None):
        """
        Simulate the motor response for the given refernce signal.

        Parameters:
        - u(t):     voltage input reference [V]
        - t_values: array of time values [s]
        - x0:       initial state [i(0), ω(0)]

        Returns:
        - i_values: armature current values
        - w_values: angular velocity values
        """
        if x0 is None:
            x0 = [0.0, 0.0]

        f = dc_motor.em_ode(u)
        solution = odeint(f, x0, t_values)

        i_values = solution[:, 0]
        w_values = solution[:, 1]

        return i_values, w_values

# simulation
i_values, w_values = simulate(motor, u, t_values)

# plot results
title = "MAXON A-max 32 24 V Brushless DC Motor\nNo Load Open Loop Simulation"
Scope.plot(title, t_values, u, i_values, w_values)
