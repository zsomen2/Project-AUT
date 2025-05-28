import numpy as np
from scipy.integrate import odeint

class Simulation:
    @staticmethod
    def simulate(dc_motor, u, t_values, x0=None):
        """
        Simulate the motor response for the given reference signal.

        Parameters:
        - dc_motor: instance of DCMotor
        - u_func: input voltage function
        - t_values: array of time values [s]
        - x0: initial state [i(0), ω(0)]

        Returns:
        - u_values: input voltage values
        - i_values: armature current values
        - w_values: angular velocity values
        """
        # set initial conditions if not provided
        if x0 is None:
            x0 = [0.0, 0.0]

        # initialize lists to store results
        i_values = []
        w_values = []
        u_values = []

        # insert initial values to align array lengths
        i_values.insert(0, x0[0])
        w_values.insert(0, x0[1])
        u_values.insert(0, u(t_values[0]))

        x = x0
        for i in range(1, len(t_values)):
            dt = [t_values[i - 1], t_values[i]]

            f = dc_motor.em_ode(u)
            x = odeint(f, x, dt)[-1]

            # append results
            u_values.append(u(t_values[i]))
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(u_values), np.array(i_values), np.array(w_values)
