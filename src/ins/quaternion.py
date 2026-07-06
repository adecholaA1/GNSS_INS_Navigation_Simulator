"""
Outils de calcul quaternion pour la navigation inertielle.

Les quaternions permettent de représenter l'orientation du véhicule
sans singularité, contrairement aux angles d'Euler.

Convention utilisée :
    q = [w, x, y, z]

où :
    w : partie scalaire
    x, y, z : partie vectorielle
"""

import numpy as np


def normalize_quaternion(q):
    """
    Normalise un quaternion.

    Parameters
    ----------
    q : ndarray (4,)
        Quaternion [w, x, y, z].

    Returns
    -------
    ndarray (4,)
        Quaternion normalisé.
    """

    norm = np.linalg.norm(q)

    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])

    return q / norm


def quaternion_multiply(q1, q2):
    """
    Calcule le produit de deux quaternions.

    Parameters
    ----------
    q1 : ndarray (4,)
        Premier quaternion.
    q2 : ndarray (4,)
        Second quaternion.

    Returns
    -------
    ndarray (4,)
        Produit q1 ⊗ q2.
    """

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def integrate_quaternion(q, angular_rate, dt):
    """
    Intègre l'orientation à partir des vitesses angulaires gyroscope.

    Parameters
    ----------
    q : ndarray (4,)
        Quaternion courant [w, x, y, z].
    angular_rate : ndarray (3,)
        Vitesse angulaire [wx, wy, wz] en rad/s.
    dt : float
        Pas de temps en secondes.

    Returns
    -------
    ndarray (4,)
        Quaternion propagé et normalisé.
    """

    omega_quat = np.array([
        0.0,
        angular_rate[0],
        angular_rate[1],
        angular_rate[2],
    ])

    q_dot = 0.5 * quaternion_multiply(q, omega_quat)

    q_next = q + q_dot * dt

    return normalize_quaternion(q_next)