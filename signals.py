import numpy as np

class SquareWave:
    """
    Square wave signal generator.
    Swithces between high and low voltage levels at a specified frequency.

    Parameters:
    - freq:   frequency of the square wave [Hz]
    - V_high: high voltage level [V]
    - V_low:  low voltage level [V]

    Methods:
    - __call__(t): evaluates the square wave at time
    """
    def __init__(self, freq, u_high, u_low):
        self.freq = freq        # frequency [Hz]
        self.u_high = u_high    # high voltage level [V]
        self.u_low = u_low      # low voltage level [V]

    def __call__(self, t, pwm=0.5):
        """
        Evaluate the square wave at specified time.
        Allows the instance to be called like a function.

        Parameters:
        - t:   time [s] (can be a scalar or an array)
        - pwm: pulse width modulation factor (default is 50%)

        Returns:
        - u(t): voltage at time t [V]
        """
        T = 1.0 / self.freq  # period [s]
        t_mod = np.mod(t, T) # position in the period

        # evaluate for scalar or array input
        if isinstance(t, (float, int)):
            return self.u_high if t_mod < (T * pwm) else self.u_low
        else:
            return np.where(t_mod < (T * pwm), self.u_high, self.u_low)
