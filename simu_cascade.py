import numpy as np

from dc_motor import DCMotor
from signals import Heaviside
from controllers import PIDController
from simulation import Simulation
from scope import Scope

from parameters import Ra, La, J, k, b

# simulation setup
duration = 0.5
dt = 1e-6

# create motor and reference signal
motor = DCMotor(Ra, La, J, k, b)
w_reference = Heaviside(150, 0.1)

# speed controller setup (outer loop)
Kp_speed = 0.16   # proportional gain
Ki_speed = 4.44   # integral gain
Kd_speed = 0      # derivative gain
freq_speed = 1e3  # controller frequency [Hz]
i_min = -5        # minimum output limit [A]
i_max = 5         # maximum output limit [A]
speed_controller = PIDController(w_reference,
                                 Kp_speed,
                                 Ki_speed,
                                 Kd_speed,
                                 freq_speed,
                                 i_min, i_max)

# current controller setup (inner loop)
Kp_current = 1.85   # proportional gain
Ki_current = 13280  # integral gain
Kd_current = 0      # derivative gain
freq_current = 1e4  # controller frequency [Hz]
u_min = 0           # minimum output limit [V]
u_max = 24          # maximum output limit [V]
current_controller = PIDController(0,
                                   Kp_current,
                                   Ki_current,
                                   Kd_current,
                                   freq_current,
                                   u_min, u_max)

# simulation
results = Simulation.simulate_cascade(motor,
                                      duration, dt,
                                      speed_controller,
                                      current_controller,
                                      None)

# plot results
title = (
    f"MAXON A-max 32 24 V DC Motor\n"
    f"Cascade Control with Different Frequencies\n"
    f"Speed: {freq_speed / 1e3:.0f} kHz, Current: {freq_current / 1e3:.0f} kHz"
)
Scope.plot(title, results, w_reference)
