"""
Mécanisation inertielle strapdown.

Ce module propage l'état inertiel à partir des mesures IMU :

    gyroscope      -> orientation
    accéléromètre  -> force spécifique body
    rotation       -> force spécifique navigation
    gravité        -> accélération navigation
    intégration    -> vitesse puis position

Convention :
    - quaternion q = [w, x, y, z]
    - repère navigation : z vers le haut
    - l'accéléromètre fournit une force spécifique body, pas une accélération navigation
"""

import numpy as np

from src.ins.quaternion import integrate_quaternion, normalize_quaternion
from src.ins.rotation import rotate_body_to_navigation
from src.ins.gravity import gravity_vector


def strapdown_step(
    position,
    velocity,
    attitude,
    specific_force_body,
    angular_rate_body,
    dt,
    reference_attitude=None,
):
    """
    Effectue un pas de mécanisation inertielle strapdown.

    Parameters
    ----------
    position : ndarray (3,)
        Position courante dans le repère navigation.

    velocity : ndarray (3,)
        Vitesse courante dans le repère navigation.

    attitude : ndarray (4,)
        Quaternion courant body vers navigation.

    specific_force_body : ndarray (3,)
        Force spécifique mesurée par l'accéléromètre dans le repère body.

    angular_rate_body : ndarray (3,)
        Vitesse angulaire gyroscope dans le repère body.

    dt : float
        Pas de temps.

    reference_attitude : ndarray (4,), optional
        Quaternion de référence disponible en simulation.
        S'il est fourni, il est utilisé pour éviter qu'une erreur d'attitude
        domine la validation de la chaîne accéléromètre → navigation.

    Returns
    -------
    position_next : ndarray (3,)
        Nouvelle position.

    velocity_next : ndarray (3,)
        Nouvelle vitesse.

    attitude_next : ndarray (4,)
        Nouvelle attitude.
    """

    if reference_attitude is None:
        attitude_next = integrate_quaternion(
            attitude,
            angular_rate_body,
            dt,
        )
    else:
        attitude_next = normalize_quaternion(reference_attitude)

    specific_force_navigation = rotate_body_to_navigation(
        specific_force_body,
        attitude_next,
    )

    acceleration_navigation = (
        specific_force_navigation
        + gravity_vector()
    )

    velocity_next = (
        velocity
        + acceleration_navigation * dt
    )

    position_next = (
        position
        + velocity * dt
        + 0.5 * acceleration_navigation * dt**2
    )

    return (
        position_next,
        velocity_next,
        attitude_next,
    )


def run_strapdown_ins(
    initial_position,
    initial_velocity,
    initial_attitude,
    accelerometer_measurements,
    gyroscope_measurements,
    dt,
    reference_attitudes=None,
):
    """
    Exécute une navigation inertielle strapdown complète.

    Parameters
    ----------
    initial_position : ndarray (3,)
        Position initiale.

    initial_velocity : ndarray (3,)
        Vitesse initiale.

    initial_attitude : ndarray (4,)
        Quaternion initial.

    accelerometer_measurements : ndarray (N,3)
        Forces spécifiques body mesurées par l'accéléromètre.

    gyroscope_measurements : ndarray (N,3)
        Mesures gyroscope dans le repère body.

    dt : float
        Pas d'échantillonnage.

    reference_attitudes : ndarray (N,4), optional
        Attitudes de référence issues du simulateur cinématique.
        Utilisées uniquement pour valider proprement la chaîne inertielle.

    Returns
    -------
    positions : ndarray (N,3)
        Positions estimées.

    velocities : ndarray (N,3)
        Vitesses estimées.

    attitudes : ndarray (N,4)
        Attitudes estimées.
    """

    n_samples = accelerometer_measurements.shape[0]

    positions = np.zeros((n_samples, 3))
    velocities = np.zeros((n_samples, 3))
    attitudes = np.zeros((n_samples, 4))

    positions[0] = initial_position
    velocities[0] = initial_velocity
    attitudes[0] = normalize_quaternion(initial_attitude)

    for k in range(1, n_samples):

        reference_attitude = None

        if reference_attitudes is not None:
            reference_attitude = reference_attitudes[k-1]

        (
            positions[k],
            velocities[k],
            attitudes[k],
        ) = strapdown_step(
            positions[k - 1],
            velocities[k - 1],
            attitudes[k - 1],
            accelerometer_measurements[k - 1],
            gyroscope_measurements[k - 1],
            dt,
            reference_attitude=reference_attitude,
        )

    return (
        positions,
        velocities,
        attitudes,
    )