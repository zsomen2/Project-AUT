from aut_project.dc_motor import DCMotor
from aut_project.signals import SquareWave
from aut_project.controllers import PIDController
from aut_project.simulation import Simulation
from aut_project.scope import Scope

from aut_project.parameters import Ra, La, J, k, b

# simulation setup
duration = 6
dt = 1e-5

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
w_reference = SquareWave(0.35, 300, 0)

# controller setup
Kp = 5.0               # proportional gain
Ki = 0.5               # integral gain
Kd = 0.05              # derivative gain
freq_controller = 1e5  # controller frequency [Hz]
u_min = 0.0            # minimum output limit [V]
u_max = 24.0           # maximum output limit [V]
controller = PIDController(w_reference, Kp, Ki, Kd, freq_controller, u_min, u_max)

# simulation
results = Simulation.simulate("closed", motor, duration, dt, controller, None)

# plot results
title = (
    f"MAXON A-max 32 24 V DC Motor\n"
    f"Closed Loop PID Control Simulation\n"
    f"Kp: {Kp}, Ki: {Ki}, Kd: {Kd}"
)
Scope.plot(title, results, w_reference)
