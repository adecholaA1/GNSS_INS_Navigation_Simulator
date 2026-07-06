"""
Navigation inertielle par intégration des mesures IMU.

Ce module reconstruit la vitesse et la position du véhicule à partir
des mesures de l'accéléromètre.

Le modèle implémenté correspond à une intégration discrète dans un
repère cartésien, en supposant que les accélérations sont déjà
exprimées dans le repère de navigation.

Cette première version constitue le cœur de la chaîne INS avant
l'introduction des modèles d'attitude (gyroscopes, matrices de rotation,
quaternions) et de la compensation de la gravité.
"""

import numpy as np


def integrate_ins(
    initial_position,
    initial_velocity,
    acceleration_measurements,
    dt,
):
    """
    Reconstruit la trajectoire par intégration inertielle.

    Les équations discrètes utilisées sont :

        v(k+1) = v(k) + a(k) Δt

        p(k+1) = p(k)
               + v(k) Δt
               + 1/2 a(k) Δt²

    Parameters
    ----------
    initial_position : ndarray (3,)
        Position initiale du véhicule.

    initial_velocity : ndarray (3,)
        Vitesse initiale du véhicule.

    acceleration_measurements : ndarray (N,3)
        Mesures de l'accéléromètre.

    dt : float
        Pas d'échantillonnage (s).

    Returns
    -------
    position : ndarray (N,3)
        Position estimée.

    velocity : ndarray (N,3)
        Vitesse estimée.
    """

    n_samples = acceleration_measurements.shape[0]

    position = np.zeros(
        (n_samples, 3),
        dtype=float
    )

    velocity = np.zeros(
        (n_samples, 3),
        dtype=float
    )

    # -----------------------------------------------------------------
    # Conditions initiales
    # -----------------------------------------------------------------

    position[0] = initial_position
    velocity[0] = initial_velocity

    # -----------------------------------------------------------------
    # Intégration inertielle
    #
    # La vitesse est obtenue par intégration de l'accélération.
    # La position est ensuite calculée à partir de la vitesse et de
    # l'accélération sur un pas d'intégration.
    #
    # Cette implémentation volontairement simple servira de base à
    # une INS complète intégrant :
    #
    #   • gyroscopes
    #   • orientation
    #   • compensation de la gravité
    #   • quaternions
    #   • navigation strapdown
    # -----------------------------------------------------------------

    for k in range(1, n_samples):

        acceleration = acceleration_measurements[k - 1]

        velocity[k] = (
            velocity[k - 1]
            + acceleration * dt
        )

        position[k] = (
            position[k - 1]
            + velocity[k - 1] * dt
            + 0.5 * acceleration * dt**2
        )

    return position, velocity