"""
Outils de rotation pour la navigation inertielle.

Ce module convertit une attitude représentée par un quaternion en matrice
de rotation. Cette matrice permet de projeter les mesures accéléromètre
du repère body vers le repère de navigation.
"""

import numpy as np

from src.ins.quaternion import normalize_quaternion


def quaternion_to_rotation_matrix(q):
    """
    Convertit un quaternion en matrice de rotation.

    Convention :
        q = [w, x, y, z]

    Parameters
    ----------
    q : ndarray (4,)
        Quaternion d'attitude.

    Returns
    -------
    ndarray (3, 3)
        Matrice de rotation body vers navigation.
    """

    q = normalize_quaternion(q)

    w, x, y, z = q

    return np.array(
        [
            [
                1.0 - 2.0 * (y**2 + z**2),
                2.0 * (x*y - z*w),
                2.0 * (x*z + y*w),
            ],
            [
                2.0 * (x*y + z*w),
                1.0 - 2.0 * (x**2 + z**2),
                2.0 * (y*z - x*w),
            ],
            [
                2.0 * (x*z - y*w),
                2.0 * (y*z + x*w),
                1.0 - 2.0 * (x**2 + y**2),
            ],
        ],
        dtype=float
    )


def rotate_body_to_navigation(vector_body, q):
    """
    Projette un vecteur du repère body vers le repère navigation.

    Parameters
    ----------
    vector_body : ndarray (3,)
        Vecteur exprimé dans le repère du drone.

    q : ndarray (4,)
        Quaternion d'attitude body vers navigation.

    Returns
    -------
    ndarray (3,)
        Vecteur exprimé dans le repère de navigation.
    """

    rotation_matrix = quaternion_to_rotation_matrix(q)

    return rotation_matrix @ vector_body