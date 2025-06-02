import numpy as np

class SquareWave:
    """
    Square wave signal generator.
    Swithces between high and low signal levels at a specified frequency.

    Parameters:
    - freq: frequency of the square wave [Hz]
    - high: high signal level
    - low:  low signal level

    Methods:
    - __call__(t): evaluates the square wave at time
    """
    def __init__(self, freq, high, low, pwm=0.5):
        self.freq = freq  # frequency [Hz]
        self.high = high  # high signal level
        self.low = low    # low signal level
        self.pwm = pwm    # pulse width modulation factor (default is 50%)

    def __call__(self, t):
        """
        Evaluate the square wave at specified time.
        Allows the instance to be called like a function.

        Parameters:
        - t: time [s] (can be a scalar or an array)

        Returns:
        - signal level evaluated at time t
        """
        T = 1.0 / self.freq  # period [s]
        t_mod = np.mod(t, T) # position in the period

        return np.where(t_mod < (T * self.pwm), self.high, self.low)

class TriangleWave:
    """
    Triangle wave signal generator.
    Creates a triangle wave oscillating between high and low signal levels at a specified frequency.

    Parameters:
    - freq: frequency of the triangle wave [Hz]
    - high: maximum value
    - low:  minimum value

    Methods:
    - __call__(t): evaluates the square wave at time
    """
    def __init__(self, freq, high, low):
        self.freq = freq  # frequency [Hz]
        self.high = high  # high signal level
        self.low = low    # low signal level

    def __call__(self, t):
        """
        Evaluate the triangle wave at specified time.
        Allows the instance to be called like a function.

        Parameters:
        - t: time [s] (can be a scalar or an array)

        Returns:
        - signal level evaluated at time t
        """
        T = 1.0 / self.freq        # period [s]
        t_mod = np.mod(t, T)       # position in the period
        amp = self.high - self.low # amplitude

        # linear interpolation for ramping
        ramping = 2 * amp * t_mod / T

        return self.low + np.where(t_mod < T/2, ramping, 2*amp - ramping)

class SineWave:
    """
    Sine wave signal generator.
    Generates a sine wave with given amplitude and offset oscillating at a specified frequency.

    Parameters:
    - freq:   frequency of the sine wave [Hz]
    - amp:    amplitude
    - offset: vertical offset (DC level)
    """
    def __init__(self, freq, amp=1.0, offset=0.0):
        self.freq = freq
        self.amp = amp
        self.offset = offset

    def __call__(self, t):
        """
        Evaluate the sine wave at specified time.
        Allows the instance to be called like a function.

        Parameters:
        - t: time [s] (can be a scalar or an array)

        Returns:
        - signal level evaluated at time t
        """
        omega = 2 * np.pi * self.freq # angular frequency [rad/s]
        
        return self.amp * np.sin(omega * t) + self.offset
