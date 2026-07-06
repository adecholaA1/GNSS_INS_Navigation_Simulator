"""
Simulation d'un gyroscope triaxial.

Le gyroscope mesure les vitesses angulaires du véhicule dans le repère
body. Ces mesures seront utilisées pour propager l'orientation de l'INS.
"""

import numpy as np


def simulate_gyroscope(
    true_angular_rate,
    sigma=0.001,
    bias=None,
    seed=42,
):
    """
    Simule les mesures d'un gyroscope 3D.

    Modèle :

        omega_mes = omega_vraie + biais + bruit

    Parameters
    ----------
    true_angular_rate : ndarray (N, 3)
        Vitesses angulaires vraies [wx, wy, wz] en rad/s.

    sigma : float
        Écart-type du bruit blanc en rad/s.

    bias : ndarray (3,), optional
        Biais constant du gyroscope en rad/s.

    seed : int, optional
        Graine aléatoire.

    Returns
    -------
    ndarray (N, 3)
        Mesures gyroscope simulées en rad/s.
    """

    rng = np.random.default_rng(seed)

    if bias is None:
        bias = np.zeros(3)

    noise = rng.normal(
        loc=0.0,
        scale=sigma,
        size=true_angular_rate.shape,
    )

    measured_angular_rate = (
        true_angular_rate
        + bias
        + noise
    )

    return measured_angular_rate