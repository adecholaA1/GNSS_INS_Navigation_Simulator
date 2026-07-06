"""
Simulation d'un accéléromètre triaxial.

Le modèle implémenté constitue une première approximation des mesures
fournies par une centrale inertielle.

La mesure est obtenue à partir de :

    a_mes = a_vraie + b + n

où :

    a_vraie : accélération réelle
    b       : biais constant
    n       : bruit blanc gaussien

L'architecture du module permet d'intégrer ultérieurement des modèles
plus réalistes tels que :

    • dérive lente du biais (bias random walk)
    • facteur d'échelle
    • désalignement des axes
    • bruit coloré
    • saturation
"""

import numpy as np


def simulate_accelerometer(
    true_acceleration,
    sigma=0.05,
    bias=None,
    seed=42,
):
    """
    Simule les mesures d'un accéléromètre 3D.

    Parameters
    ----------
    true_acceleration : ndarray (N,3)
        Accélération réelle du véhicule.

    sigma : float, optional
        Écart-type du bruit blanc (m/s²).

    bias : ndarray (3,), optional
        Biais constant appliqué aux trois axes.

    seed : int, optional
        Graine du générateur pseudo-aléatoire.

    Returns
    -------
    ndarray (N,3)
        Mesures simulées de l'accéléromètre.
    """

    rng = np.random.default_rng(seed)

    if bias is None:
        bias = np.zeros(3)

    # -----------------------------------------------------------------
    # Bruit blanc de mesure
    # -----------------------------------------------------------------

    noise = rng.normal(
        loc=0.0,
        scale=sigma,
        size=true_acceleration.shape,
    )

    # -----------------------------------------------------------------
    # Modèle simplifié de l'accéléromètre
    #
    # Les termes plus avancés (bias random walk, scale factor,
    # désalignement, etc.) seront ajoutés progressivement afin de
    # conserver une architecture modulaire.
    # -----------------------------------------------------------------

    measured_acceleration = (
        true_acceleration
        + bias
        + noise
    )

    return measured_acceleration