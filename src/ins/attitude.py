"""
Génération d'une attitude simplifiée à partir de la trajectoire.

Cette première version estime principalement le cap du drone à partir
de sa vitesse horizontale. Elle permet de produire une orientation
cohérente avec le mouvement avant d'introduire un modèle d'attitude
plus complet.
"""

import numpy as np


def compute_yaw_from_velocity(velocity):
    """
    Calcule le cap du drone à partir de la vitesse horizontale.

    Parameters
    ----------
    velocity : ndarray (N,3)
        Vitesse du drone.

    Returns
    -------
    ndarray (N,)
        Angle de lacet yaw en radians.
    """

    vx = velocity[:, 0]
    vy = velocity[:, 1]

    return np.arctan2(vy, vx)


def compute_angular_rate_from_yaw(yaw, dt):
    """
    Calcule une vitesse angulaire simplifiée à partir du lacet.

    Parameters
    ----------
    yaw : ndarray (N,)
        Angle de lacet en radians.

    dt : float
        Pas de temps.

    Returns
    -------
    ndarray (N,3)
        Vitesse angulaire [wx, wy, wz] en rad/s.
    """

    yaw_unwrapped = np.unwrap(yaw)

    yaw_rate = np.gradient(yaw_unwrapped, dt)

    angular_rate = np.zeros((len(yaw), 3))

    angular_rate[:, 2] = yaw_rate

    return angular_rate


def yaw_to_quaternion(yaw):
    """
    Convertit un angle de lacet en quaternion.

    Hypothèse :
        roll = 0
        pitch = 0

    Parameters
    ----------
    yaw : float
        Angle de lacet en radians.

    Returns
    -------
    ndarray (4,)
        Quaternion [w, x, y, z].
    """

    return np.array([
        np.cos(yaw / 2.0),
        0.0,
        0.0,
        np.sin(yaw / 2.0),
    ])