import matplotlib.pyplot as plt

class Scope:
    """
    Contains methods to plot the results of a DC motor simulation.
    """
    @staticmethod
    def plot(title, t, u, i, w):
        """
        Plot input voltage, current, and angular velocity.

        Parameters:
        - t: time array
        - u: input voltage array (can be a function or a list)
        - i: armature current array
        - w: angular velocity array
        """
        # convert u to array if it is a function
        u_values = u(t) if callable(u) else u

        plt.figure(figsize=(10, 7))

        # input voltage
        plt.subplot(3, 1, 1)
        plt.plot(t, u_values, label='Input voltage u(t) [V]', color='red')
        plt.ylabel('u(t) [V]')
        plt.grid(True)

        # armature current
        plt.subplot(3, 1, 2)
        plt.plot(t, i, label='Armature current i(t) [A]', color='blue')
        plt.ylabel('i(t) [A]')
        plt.grid(True)

        # angular velocity
        plt.subplot(3, 1, 3)
        plt.plot(t, w, label='Angular velocity ω(t) [rad/s]', color='black')
        plt.ylabel('ω(t) [rad/s]')
        plt.grid(True)

        plt.xlabel('t [s]')

        plt.suptitle(title)
        plt.tight_layout()
        plt.show()
