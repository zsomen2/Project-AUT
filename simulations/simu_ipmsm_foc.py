import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from aut_project import (
    FieldOrientedController,
    FieldOrientedDrive,
    FieldWeakeningController,
    IPMSMMotor,
    PIController,
    SinusoidalPWM,
    TwoLevelInverter,
)
from aut_project.signals import Heaviside


def main():
    # Motor and drive parameters (illustrative IPMSM taken from lab-scale drives)
    motor = IPMSMMotor(
        Rs=0.35,
        Ld=1.4e-3,
        Lq=2.6e-3,
        pole_pairs=4,
        psi_f=0.055,
        J=8.5e-4,
        B=2e-4,
        load_torque=0.2,
    )

    pwm = SinusoidalPWM(vdc=48.0, carrier_freq=10_000.0)
    inverter = TwoLevelInverter(vdc=48.0, pwm=pwm)

    speed_reference = Heaviside(value=50.0, delay=0.02)

    speed_pi = PIController(Kp=1.6, Ki=75.0, freq=2_000.0, y_min=-40.0, y_max=40.0)
    id_pi = PIController(Kp=5.0, Ki=550.0, freq=10_000.0, y_min=-20.0, y_max=20.0)
    iq_pi = PIController(Kp=5.0, Ki=550.0, freq=10_000.0, y_min=-20.0, y_max=20.0)

    field_weakening = FieldWeakeningController(
        voltage_limit=inverter.alpha_beta_limit,
        freq=id_pi.freq,
        id_min=-30.0,
        id_max=0.0,
        attack_gain=12_000.0,
        release_gain=5_000.0,
        deadband=0.01,
    )

    foc = FieldOrientedController(
        motor,
        speed_reference=speed_reference,
        speed_controller=speed_pi,
        id_controller=id_pi,
        iq_controller=iq_pi,
        id_reference=-2.0,
        voltage_limit=inverter.alpha_beta_limit,
        field_weakening=field_weakening,
    )

    drive = FieldOrientedDrive(motor, inverter, foc)
    results = drive.run(duration=0.2, dt=5e-6)

    print(f"Final mechanical speed: {results['speed'][-1]:.2f} rad/s")

    plot_results(results)


def plot_results(results):
    t = results["time"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

    axes[0].plot(t, results["speed"], label="Mechanical speed [rad/s]")
    axes[0].plot(t, results["omega_ref"], "--", label="Speed reference [rad/s]")
    axes[0].set_ylabel("Speed [rad/s]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t, results["q_currents"], label="i_q [A]")
    axes[1].plot(t, results["i_q_ref"], "--", label="i_q ref [A]")
    axes[1].plot(t, results["d_currents"], label="i_d [A]")
    axes[1].plot(t, results["i_d_ref"], "--", label="i_d ref [A]")
    axes[1].set_ylabel("Currents [A]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t, results["phase_voltages"][:, 0], label="Phase A voltage [V]")
    axes[2].plot(t, results["phase_voltages"][:, 1], label="Phase B voltage [V]")
    axes[2].plot(t, results["phase_voltages"][:, 2], label="Phase C voltage [V]")
    axes[2].set_ylabel("Voltage [V]")
    axes[2].grid(True)
    axes[2].legend()

    axes[3].plot(t, results["torque"], label="Electromagnetic torque [Nm]")
    axes[3].plot(t, results["load_torque"], "--", label="Load torque [Nm]")
    axes[3].set_ylabel("Torque [Nm]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True)
    axes[3].legend()

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
