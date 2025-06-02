import matplotlib.pyplot as plt

class Scope:
    """
    Contains methods to plot the results of the simulation.
    """
    @staticmethod
    def plot(title, results, w_ref=None):
        """
        Plot input voltage, current, and angular velocity.

        Parameters:
        - t:       time array
        - results: simulation results containing armature voltage, current and angular velocity
        - w_ref:   reference angular velocity
        """
        t = results[0]  # time values [s]
        u = results[1]  # armature voltage [V]
        i = results[2]  # armature current [A]
        w = results[3]  # angular velocity [rad/s]

        if w_ref is not None and not callable(w_ref):
            const_val = w_ref
            w_ref = lambda t: const_val

        plt.figure(figsize=(10, 7))

        # input voltage
        plt.subplot(3, 1, 1)
        plt.plot(t, u, label='Input voltage u(t) [V]', color='red')
        plt.ylabel('u(t) [V]')
        plt.legend(loc='upper right', framealpha=1.0)
        plt.grid(True)

        # armature current
        plt.subplot(3, 1, 2)
        plt.plot(t, i, label='Armature current i(t) [A]', color='blue')
        plt.ylabel('i(t) [A]')
        plt.legend(loc='upper right', framealpha=1.0)
        plt.grid(True)

        # angular velocity
        plt.subplot(3, 1, 3)
        plt.plot(t, w, label='Angular velocity ω(t) [rad/s]', color='black')
        if w_ref is not None:
            plt.plot(t, [w_ref(ti) for ti in t], '--',
                     label='Reference ω_ref(t) [rad/s]', color='gray')
        plt.ylabel('ω(t) [rad/s]')
        plt.legend(loc='upper right', framealpha=1.0)
        plt.grid(True)

        plt.xlabel('t [s]')
        plt.suptitle(title)
        plt.tight_layout()
        plt.show()
