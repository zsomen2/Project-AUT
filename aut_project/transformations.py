import numpy as np


SQRT3 = np.sqrt(3.0)


def clarke_transform(a, b, c):
    """
    Convert three-phase quantities to alpha-beta stationary frame.
    """
    alpha = (2.0 * a - b - c) / 3.0
    beta = (b - c) / SQRT3
    return alpha, beta


def inverse_clarke(alpha, beta):
    """
    Convert alpha-beta variables back to abc phase values.
    """
    a = alpha
    b = -0.5 * alpha + 0.5 * SQRT3 * beta
    c = -0.5 * alpha - 0.5 * SQRT3 * beta
    return a, b, c


def park_transform(alpha, beta, theta):
    """
    Rotate alpha-beta voltages/currents into d-q rotor aligned frame.
    """
    sin_th = np.sin(theta)
    cos_th = np.cos(theta)
    d = alpha * cos_th + beta * sin_th
    q = -alpha * sin_th + beta * cos_th
    return d, q


def inverse_park(d, q, theta):
    """
    Rotate d-q variables back to alpha-beta stationary frame.
    """
    sin_th = np.sin(theta)
    cos_th = np.cos(theta)
    alpha = d * cos_th - q * sin_th
    beta = d * sin_th + q * cos_th
    return alpha, beta
