import argparse
from dataclasses import dataclass
from typing import Dict, Tuple

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider

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


SIM_DURATION = 0.15
SIM_DT = 2.0e-5

MOTOR_PARAMS = dict(
    Rs=0.35,
    Ld=1.4e-3,
    Lq=2.6e-3,
    pole_pairs=4,
    psi_f=0.055,
    J=8.5e-4,
    B=2e-4,
    load_torque=0.2,
)

DEFAULT_SETTINGS = {
    "speed_kp": 1.6,
    "speed_ki": 75.0,
    "id_kp": 5.0,
    "id_ki": 550.0,
    "iq_kp": 5.0,
    "iq_ki": 550.0,
    "fw_attack": 12_000.0,
    "fw_release": 5_000.0,
    "speed_ref": 50.0,
    "load_torque": MOTOR_PARAMS["load_torque"],
}

SLIDER_SPECS = [
    ("speed_kp", "Speed Kp", 0.2, 5.0),
    ("speed_ki", "Speed Ki", 10.0, 200.0),
    ("id_kp", "i_d Kp", 0.5, 15.0),
    ("id_ki", "i_d Ki", 100.0, 1_200.0),
    ("iq_kp", "i_q Kp", 0.5, 15.0),
    ("iq_ki", "i_q Ki", 100.0, 1_200.0),
    ("fw_attack", "FW attack", 1_000.0, 20_000.0),
    ("fw_release", "FW release", 500.0, 10_000.0),
    ("speed_ref", "Speed ref [rad/s]", 0.0, 200.0),
    ("load_torque", "Load torque [Nm]", -2.0, 8.0),
]


def build_drive(params: Dict[str, float]) -> FieldOrientedDrive:
    motor_params = MOTOR_PARAMS.copy()
    motor_params["load_torque"] = params["load_torque"]
    motor = IPMSMMotor(**motor_params)
    pwm = SinusoidalPWM(vdc=48.0, carrier_freq=10_000.0)
    inverter = TwoLevelInverter(vdc=48.0, pwm=pwm)

    speed_pi = PIController(Kp=params["speed_kp"], Ki=params["speed_ki"], freq=2_000.0, y_min=-40.0, y_max=40.0)
    id_pi = PIController(Kp=params["id_kp"], Ki=params["id_ki"], freq=10_000.0, y_min=-20.0, y_max=20.0)
    iq_pi = PIController(Kp=params["iq_kp"], Ki=params["iq_ki"], freq=10_000.0, y_min=-20.0, y_max=20.0)

    field_weakening = FieldWeakeningController(
        voltage_limit=inverter.alpha_beta_limit,
        freq=id_pi.freq,
        id_min=-30.0,
        id_max=0.0,
        attack_gain=params["fw_attack"],
        release_gain=params["fw_release"],
        deadband=0.01,
    )

    foc = FieldOrientedController(
        motor,
        speed_reference=Heaviside(value=params["speed_ref"], delay=0.02),
        speed_controller=speed_pi,
        id_controller=id_pi,
        iq_controller=iq_pi,
        id_reference=-2.0,
        voltage_limit=inverter.alpha_beta_limit,
        field_weakening=field_weakening,
    )

    return FieldOrientedDrive(motor, inverter, foc)


def run_simulation(params: Dict[str, float]):
    drive = build_drive(params)
    return drive.run(duration=SIM_DURATION, dt=SIM_DT)


@dataclass
class PlotHandles:
    speed: Tuple
    speed_ref: Tuple
    iq: Tuple
    iq_ref: Tuple
    id: Tuple
    id_ref: Tuple
    voltages: Tuple
    torque: Tuple
    load_torque: Tuple
    axes: Tuple[plt.Axes, ...]


def init_plot(results) -> PlotHandles:
    t = results["time"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    fig.subplots_adjust(top=0.95, bottom=0.45)

    (speed_line,) = axes[0].plot(t, results["speed"], label="Mechanical speed [rad/s]")
    (speed_ref_line,) = axes[0].plot(t, results["omega_ref"], "--", label="Speed reference [rad/s]")
    axes[0].set_ylabel("Speed [rad/s]")
    axes[0].grid(True)
    axes[0].legend()

    (iq_line,) = axes[1].plot(t, results["q_currents"], label="i_q [A]")
    (iq_ref_line,) = axes[1].plot(t, results["i_q_ref"], "--", label="i_q ref [A]")
    (id_line,) = axes[1].plot(t, results["d_currents"], label="i_d [A]")
    (id_ref_line,) = axes[1].plot(t, results["i_d_ref"], "--", label="i_d ref [A]")
    axes[1].set_ylabel("Currents [A]")
    axes[1].grid(True)
    axes[1].legend()

    voltage_lines = axes[2].plot(
        t, results["phase_voltages"][:, 0], label="Phase A [V]"
    )
    voltage_lines += axes[2].plot(t, results["phase_voltages"][:, 1], label="Phase B [V]")
    voltage_lines += axes[2].plot(t, results["phase_voltages"][:, 2], label="Phase C [V]")
    axes[2].set_ylabel("Voltage [V]")
    axes[2].grid(True)
    axes[2].legend()

    (torque_line,) = axes[3].plot(t, results["torque"], label="Electromagnetic torque [Nm]")
    (load_torque_line,) = axes[3].plot(
        t, results["load_torque"], "--", label="Load torque [Nm]"
    )
    axes[3].set_ylabel("Torque [Nm]")
    axes[3].set_xlabel("Time [s]")
    axes[3].grid(True)
    axes[3].legend()

    return PlotHandles(
        speed=(speed_line,),
        speed_ref=(speed_ref_line,),
        iq=(iq_line,),
        iq_ref=(iq_ref_line,),
        id=(id_line,),
        id_ref=(id_ref_line,),
        voltages=tuple(voltage_lines),
        torque=(torque_line,),
        load_torque=(load_torque_line,),
        axes=tuple(axes),
    )


def update_plot(handles: PlotHandles, results):
    t = results["time"]
    handles.speed[0].set_data(t, results["speed"])
    handles.speed_ref[0].set_data(t, results["omega_ref"])
    handles.iq[0].set_data(t, results["q_currents"])
    handles.iq_ref[0].set_data(t, results["i_q_ref"])
    handles.id[0].set_data(t, results["d_currents"])
    handles.id_ref[0].set_data(t, results["i_d_ref"])
    for line, column in zip(handles.voltages, results["phase_voltages"].T):
        line.set_data(t, column)
    handles.torque[0].set_data(t, results["torque"])
    handles.load_torque[0].set_data(t, results["load_torque"])

    for ax in handles.axes:
        ax.relim()
        ax.autoscale_view()


def add_sliders(fig, params):
    sliders = {}
    cols = 2
    width = 0.32
    height = 0.03
    v_spacing = 0.045
    h_spacing = 0.15
    left_start = 0.08
    bottom_start = 0.38

    for idx, (key, label, vmin, vmax) in enumerate(SLIDER_SPECS):
        col = idx % cols
        row = idx // cols
        left = left_start + col * (width + h_spacing)
        bottom = bottom_start - row * v_spacing
        ax = fig.add_axes([left, bottom, width, height])
        sliders[key] = Slider(
            ax=ax,
            label=label,
            valmin=vmin,
            valmax=vmax,
            valinit=params[key],
        )
    return sliders


def launch_interactive():
    params = DEFAULT_SETTINGS.copy()
    results = run_simulation(params)
    handles = init_plot(results)
    fig = handles.axes[0].figure

    sliders = add_sliders(fig, params)

    def on_slider_change(_):
        for key, slider in sliders.items():
            params[key] = slider.val
        new_results = run_simulation(params)
        update_plot(handles, new_results)
        fig.canvas.draw_idle()

    for slider in sliders.values():
        slider.on_changed(on_slider_change)

    reset_ax = fig.add_axes([0.82, 0.06, 0.12, 0.04])
    button = Button(reset_ax, "Reset gains", hovercolor="0.8")

    def on_reset(_event):
        for slider in sliders.values():
            slider.reset()

    button.on_clicked(on_reset)

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Interactive IPMSM FOC tuner.")
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run a single simulation with default gains (no GUI, useful for CI/testing).",
    )
    args = parser.parse_args()

    if args.headless:
        results = run_simulation(DEFAULT_SETTINGS)
        print(f"Final mechanical speed: {results['speed'][-1]:.2f} rad/s")
        return

    launch_interactive()


if __name__ == "__main__":
    main()
