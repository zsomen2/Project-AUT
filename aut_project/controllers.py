class PIDController:
    """
    PID Controller for a control loop.

    Parameters:
    - setpoint: desired reference value
    - Kp:       proportional gain
    - Ki:       integral gain
    - Kd:       derivative gain
    - dt:       time step
    - y_min:    minimum output limit
    - y_max:    maximum output limit

    Methods:
    - reset: resets the internal state
    - calculate: calculates the PID control signal based on the feedback value
    """
    def __init__(self, setpoint, Kp, Ki, Kd, freq, y_min=float('-inf'), y_max=float('inf')):
        self.setpoint = setpoint if callable(setpoint) else lambda t: setpoint
        self.Kp = Kp           # proportional gain
        self.Ki = Ki           # integral gain
        self.Kd = Kd           # derivative gain
        self.freq = freq       # controller frequency
        self.dt = 1/freq       # time step duration
        self.y_min = y_min     # minimum output limit
        self.y_max = y_max     # maximum output limit

        self.integral = 0.0    # integral term with memory
        self.prev_error = 0.0  # previous error for derivative calculation

    def reset(self):
        """
        Reset the internal state.
        """
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, measured_value, t):
        """
        Calculate the PID control signal.

        Parameters:
        - measured_value: current value from the system
        - t:              current time

        Returns:
        - y: saturated control output signal
        """
        # calculate the error from feedback
        error = self.setpoint(t) - measured_value

        # calculate the PID terms
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt if t != 0 else 0.0
        self.prev_error = error

        # calculate the control output
        y = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # anti-windup
        if y > self.y_max:
            y = self.y_max
            self.integral -= error * self.dt
        elif y < self.y_min:
            y = self.y_min
            self.integral -= error * self.dt

        # saturated output
        return y
