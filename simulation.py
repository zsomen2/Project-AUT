import numpy as np
from scipy.integrate import odeint

class Simulation:
    """
    Methods for simulating the response of a DC motor

    Methods:
    - simulate: simulates the motor response for a given reference signal
    """
    @staticmethod
    def simulate(dc_motor, t_values, x0=None, u_reference=None, controller=None):
        """
        Parameters:
        - dc_motor:     instance of DCMotor
        - t_values:     time array [s]
        - x0:           initial conditions [i(0), ω(0)]
        - u_reference:  input reference voltage
        - controller:   reference angular velocity from the controller

        Returns:
        - u_values: armature voltage values
        - i_values: armature current values
        - w_values: angular velocity values
        """
        # initialize lists for results
        i_values = []
        w_values = []
        u_values = []

        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = x0

        # initial voltage
        if controller:
            u0 = controller.calculate(x0[1], t_values[0])
        elif u_reference:
            u0 = u_reference(t_values[0])
        else:
            raise ValueError("No reference signal provided!")

        # store initial state
        u_values.append(u0)
        i_values.append(x0[0])
        w_values.append(x0[1])

        # simulate the motor response

        if u_reference:
            for i in range(1, len(t_values)):
                dt = [t_values[i - 1], t_values[i]]

                # simulate step by step
                f = dc_motor.em_ode(u_reference)
                x = odeint(f, x, dt)[-1]

                # append results
                u_values.append(u_reference(t_values[i]))
                i_values.append(x[0])
                w_values.append(x[1])
        else:
            for i in range(1, len(t_values)):
                # angular velocity for feedback
                w = x[1]

                # input voltage from the controller
                u = controller.calculate(w, t_values[i])

                # simulate step by step
                f = dc_motor.em_ode(lambda _: u)
                x = odeint(f, x, [t_values[i], t_values[i] + controller.dt])[-1]

                # append results
                u_values.append(u)
                i_values.append(x[0])
                w_values.append(x[1])

        return np.array(u_values), np.array(i_values), np.array(w_values)
