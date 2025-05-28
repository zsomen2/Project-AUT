import numpy as np

class SquareWave:
    """
    Square wave signal generator.
    Swithces between high and low signal levels at a specified frequency.

    Parameters:
    - freq:   frequency of the square wave [Hz]
    - high: high signal level
    - low:  low signal level

    Methods:
    - __call__(t): evaluates the square wave at time
    """
    def __init__(self, freq, high, low):
        self.freq = freq    # frequency [Hz]
        self.high = high  # high signal level
        self.low = low    # low signal level

    def __call__(self, t, pwm=0.5):
        """
        Evaluate the square wave at specified time.
        Allows the instance to be called like a function.

        Parameters:
        - t:   time [s] (can be a scalar or an array)
        - pwm: pulse width modulation factor (default is 50%)

        Returns:
        - signal level evaluated at time t
        """
        T = 1.0 / self.freq  # period [s]
        t_mod = np.mod(t, T) # position in the period

        # evaluate for scalar or array input
        if isinstance(t, (float, int)):
            return self.high if t_mod < (T * pwm) else self.low
        else:
            return np.where(t_mod < (T * pwm), self.high, self.low)
