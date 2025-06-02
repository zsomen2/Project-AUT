class DCMotor:
    """
    DC Motor model

    Parameters:
    - Ra: armature resistance [Ohm]
    - La: armature inductance [H]
    - J:  rotor inertia [kg*m^2]
    - k:  motor constant [V/rad/s]
    - b:  viscous friction coefficient [N*m*s/rad]
    - T:  load torque [N*m]

    Methods:
    - em_ode: returns the electromechanical ODE for the motor
    """
    def __init__(self, Ra, La, J, k, b=0, T=0):
        self.Ra = Ra  # armature resistance          [Ohm]
        self.La = La  # armature inductance          [H]
        self.J = J    # rotor inertia                [kg*m^2]
        self.k = k    # motor constant               [V/rad/s]
        self.b = b    # viscous friction coefficient [N*m*s/rad]
        self.T = T    # load torque                  [N*m]

    def em_ode(self, u):
        """
        Electromechanical ODE for the DC motor.

        Parameters:
        - x[0]: current [A]
        - x[1]: angular velocity [rad/s]
        - u(t): voltage reference input [V]

        Returns:
        - f: function of the derivatives [di/dt, dw/dt]
        """
        def f(x, t):
            i, w = x
            di_dt = (u(t) - self.Ra * i - self.k * w) / self.La
            dw_dt = (self.k * i - self.b * w - self.T) / self.J
            return [di_dt, dw_dt]
        
        return f
