import numpy as np

class Simulation:
    """
    Methods for simulating the response of a DC motor.

    Methods:
    - simulate: dispatch simulation based on mode
    - simulate_open_loop: open-loop simulation with a reference voltage signal
    - simulate_closed_loop: closed-loop simulation with a controller
    - simulate_cascade: cascade control simulation
    """
    @staticmethod
    def simulate(mode, dc_motor, duration, dt, *args):
        """
        Dispatch simulation based on mode.

        Parameters:
        - mode:      simulation mode
        - dc_motor:  instance of DCMotor
        - duration:  simulation duration [s]
        - dt:        time step [s]
        - *args:     additional arguments depending on mode

        Returns:
        - simulation results
        """
        if mode == "open":
            u_reference = args[0]
            x0 = args[1] if len(args) > 1 else None
            return Simulation.simulate_open_loop(dc_motor,
                                                 duration, dt,
                                                 u_reference,
                                                 x0)

        elif mode == "closed":
            controller = args[0]
            x0 = args[1] if len(args) > 1 else None
            return Simulation.simulate_closed_loop(dc_motor,
                                                   duration, dt,
                                                   controller,
                                                   x0)

        elif mode == "cascade":
            speed_controller = args[0]
            current_controller = args[1]
            x0 = args[2] if len(args) > 2 else None
            return Simulation.simulate_cascade(dc_motor,
                                               duration, dt,
                                               speed_controller,
                                               current_controller,
                                               x0)

        else:
            raise ValueError(f"Unknown simulation mode: {mode}")

    @staticmethod
    def simulate_open_loop(dc_motor, duration, dt, u_reference, x0=None):
        """
        Open-loop simulation with a reference voltage signal.

        Parameters:
        - dc_motor:     instance of a DCMotor class
        - duration:     total simulation time [s]
        - dt:           time step [s]
        - u_reference:  input reference voltage
        - x0:           initial state [i(0), w(0)]

        Returns:
        - t_values: time values
        - u_values: armature voltage values
        - i_values: armature current values
        - w_values: angular velocity values
        """
        # initialize time values
        t_values = np.arange(0, duration, dt)

        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = np.array(x0, dtype=float)

        # initialize lists for results
        u_values = []
        i_values = []
        w_values = []

        # simulation loop
        for j in range(len(t_values)):
            # simulate using Euler integration
            f = dc_motor.em_ode(u_reference)
            dxdt = f(x, t_values[j])
            x += dt * np.array(dxdt)

            # append results
            u_values.append(u_reference(t_values[j]))
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(t_values), np.array(u_values), np.array(i_values), np.array(w_values)

    @staticmethod
    def simulate_closed_loop(dc_motor, duration, dt, controller, x0=None):
        """
        Closed-loop simulation with a controller.

        Parameters:
        - dc_motor:    instance of a DCMotor class
        - duration:    total simulation time [s]
        - dt:          time step [s]
        - controller:  instance of a controller class
        - x0:          initial state [i(0), w(0)]

        Returns:
        - t_values: time values
        - u_values: armature voltage values
        - i_values: armature current values
        - w_values: angular velocity values
        """
        # initialize time values
        t_values = np.arange(0, duration, dt)

        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = np.array(x0, dtype=float)

        # initialize lists for results
        u_values = []
        i_values = []
        w_values = []

        # reset the controller
        controller.reset()

        # update interval for the controller
        update_interval = max(1, int(round(controller.dt / dt)))

        # simulation loop
        for j in range(len(t_values)):
            # zero-order hold
            if j % update_interval == 0:
                u = controller.calculate(x[1], t_values[j])

            # Euler integration
            f = dc_motor.em_ode(lambda _: u)
            dxdt = f(x, t_values[j])
            x += dt * np.array(dxdt)

            u_values.append(u)
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(t_values), np.array(u_values), np.array(i_values), np.array(w_values)

    @staticmethod
    def simulate_cascade(dc_motor, duration, dt, speed_controller, current_controller, x0=None):
        """
        Cascade control simulation.

        Parameters:
        - dc_motor:            instance of a DCMotor class
        - duration:            total simulation time [s]
        - dt:                  time step [s]
        - speed_controller:    instance of a controller class
        - current_controller:  instance of a controller class
        - x0:                  initial state [i(0), w(0)]

        Returns:
        - t_values: time values
        - u_values: armature voltage values
        - i_values: armature current values
        - w_values: angular velocity values
        """
        # initialize time values
        t_values = np.arange(0, duration, dt)

        # initical conditions
        if x0 is None:
            x0 = [0.0, 0.0]
        x = np.array(x0, dtype=float)

        # Initialize result storage
        u_values = []
        i_values = []
        w_values = []

        # reset the controllers
        speed_controller.reset()
        current_controller.reset()

        # update intervals
        update_speed = max(1, int(round(speed_controller.dt / dt)))
        update_current = max(1, int(round(current_controller.dt / dt)))

        for j in range(len(t_values)):
            # zero-order hold for outer loop
            if j % update_speed == 0:
                i_reference = speed_controller.calculate(x[1], t_values[j])

            # zero-order hold for inner loop
            if j % update_current == 0:
                current_controller.setpoint = lambda t: i_reference
                u = current_controller.calculate(x[0], t_values[j])

            # Euler integration
            f = dc_motor.em_ode(lambda _: u)
            dxdt = f(x, t_values[j])
            x += dt * np.array(dxdt)

            # Store results
            u_values.append(u)
            i_values.append(x[0])
            w_values.append(x[1])

        return np.array(t_values), np.array(u_values), np.array(i_values), np.array(w_values)
