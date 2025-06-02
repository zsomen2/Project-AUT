import numpy as np

class Simulation:
    """
    Methods for simulating the response of a DC motor using Euler integration.

    Methods:
    - simulate_open_loop: simulates with a reference input voltage signal
    - simulate_closed_loop: simulates with a controller loop (e.g. PID)
    """
    @staticmethod
    def simulate_open_loop(dc_motor, t_values, u_reference, x0=None):
        """
        Open-loop simulation with a reference voltage signal.

        Parameters:
        - dc_motor:     instance of a DCMotor class
        - t_values:     time array [s]
        - u_reference:  input reference voltage
        - x0:           initial state [i(0), w(0)]

        Returns:
        - u_values: list of voltage values
        - i_values: list of current values
        - w_values: list of speed values
        """
        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = np.array(x0, dtype=float)

        # initialize lists for results
        u_values = [u_reference(t_values[0])]
        i_values = [x[0]]
        w_values = [x[1]]

        for i in range(1, len(t_values)):
            # simulate using Euler integration
            f = dc_motor.em_ode(u_reference)
            dxdt = f(x, t_values[i])
            dt = t_values[i] - t_values[i - 1]
            x += dt * np.array(dxdt)

            # append results
            u_values.append(u_reference(t_values[i]))
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(u_values), np.array(i_values), np.array(w_values)

    @staticmethod
    def simulate_closed_loop(dc_motor, t_values, controller, x0=None):
        """
        Closed-loop simulation with a controller (e.g., PID).

        Parameters:
        - dc_motor:    instance of DCMotor
        - t_values:    time array [s]
        - controller:  instance of a controller class
        - x0:          initial state [i(0), w(0)]

        Returns:
        - u_values: list of voltage values
        - i_values: list of current values
        - w_values: list of speed values
        """
        # reset the controller
        controller.reset()

        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = np.array(x0, dtype=float)
        
        # initialize lists for results
        u_values = [controller.calculate(x[1], t_values[0])]
        i_values = [x[0]]
        w_values = [x[1]]

        for i in range(1, len(t_values)):
            # angular velocity for feedback
            w = x[1]

            # input voltage from the controller
            u = controller.calculate(w, t_values[i])

            # simulate using Euler integration
            f = dc_motor.em_ode(lambda _: u)            
            dxdt = f(x, t_values[i])
            dt = t_values[i] - t_values[i - 1]
            x += dt * np.array(dxdt)

            u_values.append(u)
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(u_values), np.array(i_values), np.array(w_values)
