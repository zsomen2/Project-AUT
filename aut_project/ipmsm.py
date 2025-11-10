import numpy as np

from .transformations import clarke_transform, park_transform, inverse_park, inverse_clarke


class IPMSMMotor:
    """
    Interior PMSM model in the d-q reference frame.
    """

    def __init__(self, Rs, Ld, Lq, pole_pairs, psi_f, J, B=0.0, load_torque=0.0):
        self.Rs = Rs
        self.Ld = Ld
        self.Lq = Lq
        self.pole_pairs = pole_pairs
        self.psi_f = psi_f
        self.J = J
        self.B = B
        self.load_torque = load_torque

    def initial_state(self, id0=0.0, iq0=0.0, omega0=0.0, theta0=0.0):
        return np.array([id0, iq0, omega0, theta0], dtype=float)

    def electrical_speed(self, state):
        return self.pole_pairs * state[2]

    def electromagnetic_torque(self, id_value, iq_value):
        return 1.5 * self.pole_pairs * (self.psi_f * iq_value + (self.Ld - self.Lq) * id_value * iq_value)

    def derivatives(self, state, v_abc):
        id_value, iq_value, omega_m, theta_e = state
        omega_e = self.pole_pairs * omega_m

        v_alpha, v_beta = clarke_transform(*v_abc)
        v_d, v_q = park_transform(v_alpha, v_beta, theta_e)

        di_d = (v_d - self.Rs * id_value + omega_e * self.Lq * iq_value) / self.Ld
        di_q = (v_q - self.Rs * iq_value - omega_e * (self.Ld * id_value + self.psi_f)) / self.Lq

        torque = self.electromagnetic_torque(id_value, iq_value)
        d_omega = (torque - self.B * omega_m - self.load_torque) / self.J
        d_theta = omega_e

        return np.array([di_d, di_q, d_omega, d_theta])

    def phase_currents(self, state):
        i_alpha, i_beta = inverse_park(state[0], state[1], state[3])
        return inverse_clarke(i_alpha, i_beta)

    def dq_currents(self, state):
        return state[0], state[1]
