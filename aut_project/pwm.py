import numpy as np


class SinusoidalPWM:
    """
    Ideal sinusoidal PWM model that limits requested voltages to the DC bus.
    """

    def __init__(self, vdc, carrier_freq):
        self.vdc = vdc
        self.carrier_freq = carrier_freq
        self._v_limit = 0.5 * vdc

    @property
    def voltage_limit(self):
        """Maximum achievable phase voltage."""
        return self._v_limit

    def apply(self, v_ref, _t):
        """
        Clip the requested phase voltage to the achievable range and
        return both the voltage and the implied duty-cycle.
        """
        v_actual = np.clip(v_ref, -self._v_limit, self._v_limit)
        duty = 0.5 + v_actual / self.vdc
        duty = float(np.clip(duty, 0.0, 1.0))
        return v_actual, duty
