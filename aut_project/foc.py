import numpy as np

from .transformations import clarke_transform, park_transform, inverse_park


class FieldWeakeningController:
    """
    Simple field-weakening unit that adjusts the d-axis current command
    when the commanded voltage magnitude approaches the inverter limit.
    """

    def __init__(
        self,
        voltage_limit,
        freq,
        id_min=-60.0,
        id_max=0.0,
        attack_gain=10_000.0,
        release_gain=4_000.0,
        deadband=0.02,
    ):
        self.voltage_limit = voltage_limit
        self.dt = 1.0 / freq
        self.id_min = id_min
        self.id_max = id_max
        self.attack_gain = attack_gain
        self.release_gain = release_gain
        self.deadband = deadband
        self._id_cmd = None

    def reset(self):
        """Forget the previous command (used at the beginning of simulations)."""
        self._id_cmd = None

    def command(self, base_id):
        """Return the currently scheduled d-axis reference."""
        if self._id_cmd is None:
            self._id_cmd = float(np.clip(base_id, self.id_min, self.id_max))
        return self._id_cmd

    def update(self, base_id, voltage_magnitude):
        """
        Update the internal d-axis request based on the latest voltage magnitude.
        """
        if self.voltage_limit is None or self.voltage_limit <= 0.0:
            self._id_cmd = float(np.clip(base_id, self.id_min, self.id_max))
            return self._id_cmd

        if self._id_cmd is None:
            self._id_cmd = float(np.clip(base_id, self.id_min, self.id_max))

        limit = self.voltage_limit
        margin = (voltage_magnitude - limit) / limit

        if margin > self.deadband:
            # Over the limit -> push id more negative to weaken the field.
            self._id_cmd -= self.attack_gain * margin * self.dt
        elif margin < -self.deadband:
            # Below the limit by a safe margin -> relax back toward the base id.
            self._id_cmd += self.release_gain * (base_id - self._id_cmd) * self.dt
        # Within deadband -> keep the current command.

        self._id_cmd = float(np.clip(self._id_cmd, self.id_min, self.id_max))
        return self._id_cmd


class FieldOrientedController:
    """
    Field oriented controller for an IPMSM motor consisting of:
    - outer speed PI loop
    - inner d-q current PI loops with decoupling/feedforward terms
    """

    def __init__(
        self,
        motor,
        speed_reference,
        speed_controller,
        id_controller,
        iq_controller,
        id_reference=0.0,
        voltage_limit=None,
        field_weakening=None,
        torque_reference=None,
    ):
        self.motor = motor
        self.speed_reference = (
            speed_reference if callable(speed_reference) else lambda t: speed_reference
        )
        self._id_reference = id_reference if callable(id_reference) else (lambda t: id_reference)
        self.speed_controller = speed_controller
        self.id_controller = id_controller
        self.iq_controller = iq_controller
        self.voltage_limit = voltage_limit
        self.field_weakening = field_weakening
        if torque_reference is None:
            self.torque_reference = lambda _t: 0.0
        elif callable(torque_reference):
            self.torque_reference = torque_reference
        else:
            self.torque_reference = lambda _t, val=torque_reference: val
        self._speed_next_update = 0.0
        self._id_next_update = 0.0
        self._iq_next_update = 0.0
        self._fw_next_update = 0.0
        self._omega_ref_last = 0.0
        self._iq_ref_last = 0.0
        self._id_ref_cmd = None
        self._v_d_pi = 0.0
        self._v_q_pi = 0.0
        self._torque_ref_last = 0.0
        self._iq_total_ref = 0.0

    def reset(self):
        self.speed_controller.reset()
        self.id_controller.reset()
        self.iq_controller.reset()
        if self.field_weakening is not None:
            self.field_weakening.reset()
        self._speed_next_update = 0.0
        self._id_next_update = 0.0
        self._iq_next_update = 0.0
        self._fw_next_update = (
            self.field_weakening.dt if self.field_weakening is not None else 0.0
        )
        self._omega_ref_last = 0.0
        self._iq_ref_last = 0.0
        self._id_ref_cmd = None
        self._v_d_pi = 0.0
        self._v_q_pi = 0.0
        self._torque_ref_last = 0.0
        self._iq_total_ref = 0.0

    def step(self, phase_currents, theta_e, omega_m, t):
        i_alpha, i_beta = clarke_transform(*phase_currents)
        i_d, i_q = park_transform(i_alpha, i_beta, theta_e)

        base_id = self._id_reference(t)
        if self.field_weakening is not None:
            if self._id_ref_cmd is None:
                self._id_ref_cmd = self.field_weakening.command(base_id)
        else:
            self._id_ref_cmd = base_id

        if t >= self._speed_next_update - 1e-12:
            self._omega_ref_last = self.speed_reference(t)
            self.speed_controller.set_reference(self._omega_ref_last)
            self._iq_ref_last = self.speed_controller.calculate(omega_m, t)
            self._speed_next_update = t + self.speed_controller.dt

        torque_ref = self.torque_reference(t)
        self._torque_ref_last = torque_ref
        id_for_torque = self._id_ref_cmd if self._id_ref_cmd is not None else base_id
        denom = 1.5 * self.motor.pole_pairs * (
            self.motor.psi_f + (self.motor.Ld - self.motor.Lq) * id_for_torque
        )
        if abs(denom) < 1e-9:
            iq_from_torque = 0.0
        else:
            iq_from_torque = torque_ref / denom
        self._iq_total_ref = self._iq_ref_last + iq_from_torque

        if t >= self._id_next_update - 1e-12:
            self.id_controller.set_reference(self._id_ref_cmd)
            self._v_d_pi = self.id_controller.calculate(i_d, t)
            self._id_next_update = t + self.id_controller.dt

        if t >= self._iq_next_update - 1e-12:
            self.iq_controller.set_reference(self._iq_total_ref)
            self._v_q_pi = self.iq_controller.calculate(i_q, t)
            self._iq_next_update = t + self.iq_controller.dt

        omega_e = self.motor.pole_pairs * omega_m

        v_d_ff = self._v_d_pi - omega_e * self.motor.Lq * i_q
        v_q_ff = self._v_q_pi + omega_e * (self.motor.Ld * i_d + self.motor.psi_f)

        magnitude = float(np.hypot(v_d_ff, v_q_ff))
        v_d = v_d_ff
        v_q = v_q_ff
        saturated = False

        if self.voltage_limit is not None and self.voltage_limit > 0.0:
            if magnitude > self.voltage_limit:
                scale = self.voltage_limit / magnitude
                v_d *= scale
                v_q *= scale
                saturated = True

        if self.field_weakening is not None and t >= self._fw_next_update - 1e-12:
            self._id_ref_cmd = self.field_weakening.update(base_id, magnitude)
            self._fw_next_update = t + self.field_weakening.dt

        v_alpha, v_beta = inverse_park(v_d, v_q, theta_e)

        debug = {
            "i_d": i_d,
            "i_q": i_q,
            "i_q_ref": self._iq_total_ref,
            "i_d_ref": self._id_ref_cmd if self._id_ref_cmd is not None else base_id,
            "omega_ref": self._omega_ref_last,
            "v_d": v_d,
            "v_q": v_q,
            "voltage_magnitude": magnitude,
            "voltage_saturated": saturated,
            "torque_ref": self._torque_ref_last,
        }
        return (v_alpha, v_beta), debug


class FieldOrientedDrive:
    """
    Glue logic tying the controller, inverter and motor model together.
    """

    def __init__(self, motor, inverter, controller):
        self.motor = motor
        self.inverter = inverter
        self.controller = controller

    def run(self, duration, dt, x0=None):
        steps = int(duration / dt)
        t_values = np.linspace(0.0, duration, steps, endpoint=False)
        state = self.motor.initial_state() if x0 is None else np.array(x0, dtype=float)
        phase_currents = self.motor.phase_currents(state)

        results = {
            "time": t_values,
            "phase_voltages": np.zeros((steps, 3)),
            "phase_currents": np.zeros((steps, 3)),
            "d_currents": np.zeros(steps),
            "q_currents": np.zeros(steps),
            "speed": np.zeros(steps),
            "electrical_angle": np.zeros(steps),
            "torque": np.zeros(steps),
            "duty_cycles": np.zeros((steps, 3)),
            "i_q_ref": np.zeros(steps),
            "i_d_ref": np.zeros(steps),
            "omega_ref": np.zeros(steps),
            "voltage_magnitude": np.zeros(steps),
            "voltage_saturated": np.zeros(steps, dtype=bool),
            "torque_ref": np.zeros(steps),
            "load_torque": np.zeros(steps),
        }

        self.controller.reset()

        for idx, t in enumerate(t_values):
            v_alpha_beta, debug = self.controller.step(phase_currents, state[3], state[2], t)
            phase_voltages, duty = self.inverter.apply(v_alpha_beta, t)

            derivatives = self.motor.derivatives(state, phase_voltages)
            state = state + dt * derivatives
            state[3] = np.mod(state[3], 2.0 * np.pi)
            phase_currents = self.motor.phase_currents(state)

            torque = self.motor.electromagnetic_torque(state[0], state[1])

            results["phase_voltages"][idx] = phase_voltages
            results["phase_currents"][idx] = phase_currents
            results["d_currents"][idx] = debug["i_d"]
            results["q_currents"][idx] = debug["i_q"]
            results["speed"][idx] = state[2]
            results["electrical_angle"][idx] = state[3]
            results["torque"][idx] = torque
            results["duty_cycles"][idx] = duty
            results["i_q_ref"][idx] = debug["i_q_ref"]
            results["i_d_ref"][idx] = debug["i_d_ref"]
            results["omega_ref"][idx] = debug["omega_ref"]
            results["voltage_magnitude"][idx] = debug["voltage_magnitude"]
            results["voltage_saturated"][idx] = debug["voltage_saturated"]
            results["torque_ref"][idx] = debug["torque_ref"]
            results["load_torque"][idx] = self.motor.load_torque

        return results
