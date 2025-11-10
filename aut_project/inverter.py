import numpy as np

from .transformations import inverse_clarke


class TwoLevelInverter:
    """
    Two-level, three-phase inverter driven by an external PWM model.
    """

    def __init__(self, vdc, pwm):
        self.vdc = vdc
        self.pwm = pwm

    @property
    def alpha_beta_limit(self):
        """
        Maximum achievable voltage magnitude in the alpha-beta plane.
        For sinusoidal PWM the fundamental phase voltage amplitude
        equals the half DC-link voltage, so we expose that limit.
        """
        return self.pwm.voltage_limit

    def apply(self, v_alpha_beta, t):
        """
        Convert alpha-beta voltages into abc phases, feed them to the PWM
        model and return both the realized phase voltages and duty ratios.
        """
        v_alpha, v_beta = v_alpha_beta
        phases_ref = inverse_clarke(v_alpha, v_beta)

        phases = []
        duties = []
        for v_ref in phases_ref:
            v_actual, duty = self.pwm.apply(v_ref, t)
            phases.append(v_actual)
            duties.append(duty)

        return tuple(phases), tuple(duties)
