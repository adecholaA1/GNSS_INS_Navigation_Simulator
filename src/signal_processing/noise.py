"""
Modèles simplifiés des erreurs de mesure GNSS.

Chaque fonction représente une source d'erreur indépendante pouvant être
combinée avec les autres afin de construire différents scénarios de
simulation.

Ce module est conçu pour évoluer progressivement vers des modèles plus
réalistes : retards atmosphériques, pertes satellites, jamming, spoofing,
multipath urbain, etc.
"""

import numpy as np


def gaussian_noise(shape, sigma=2.0, seed=None):
    """
    Génère un bruit blanc gaussien.

    Parameters
    ----------
    shape : tuple
        Dimensions du bruit à générer.
    sigma : float, optional
        Écart-type du bruit en mètres.
    seed : int, optional
        Graine aléatoire pour rendre la simulation reproductible.

    Returns
    -------
    ndarray
        Bruit gaussien en mètres.
    """

    rng = np.random.default_rng(seed)

    return rng.normal(
        loc=0.0,
        scale=sigma,
        size=shape
    )


def constant_bias(shape, bias=10.0):
    """
    Génère un biais constant appliqué à toutes les mesures.

    Parameters
    ----------
    shape : tuple
        Dimensions de la matrice de biais.
    bias : float
        Valeur du biais en mètres.

    Returns
    -------
    ndarray
        Biais constant en mètres.
    """

    return np.full(
        shape,
        bias,
        dtype=float
    )


def satellite_bias(shape, satellite_index=0, bias=30.0):
    """
    Génère un biais affectant un seul satellite.

    Ce modèle permet de simuler une anomalie localisée sur une mesure GNSS.
    Il sera utile pour tester plus tard les méthodes d'intégrité comme RAIM.

    Parameters
    ----------
    shape : tuple
        Dimensions de la matrice de mesures.
    satellite_index : int
        Indice du satellite affecté.
    bias : float
        Valeur du biais en mètres.

    Returns
    -------
    ndarray
        Matrice contenant le biais du satellite sélectionné.
    """

    noise = np.zeros(
        shape,
        dtype=float
    )

    noise[:, satellite_index] = bias

    return noise


def generate_multipath(t, n_satellites, amplitudes=None, omega=0.05):
    """
    Génère un multipath sinusoïdal simplifié.

    Le multipath est modélisé comme une perturbation périodique propre à
    chaque satellite, avec des amplitudes et des phases différentes.

    Parameters
    ----------
    t : ndarray
        Temps de simulation.
    n_satellites : int
        Nombre de satellites.
    amplitudes : ndarray, optional
        Amplitude du multipath pour chaque satellite en mètres.
    omega : float
        Pulsation du multipath en rad/s.

    Returns
    -------
    ndarray
        Erreurs de multipath en mètres, de forme (len(t), n_satellites).
    """

    if amplitudes is None:
        amplitudes = np.linspace(
            2.0,
            8.0,
            n_satellites
        )

    phases = np.linspace(
        0.0,
        2.0 * np.pi,
        n_satellites,
        endpoint=False
    )

    multipath = np.zeros(
        (len(t), n_satellites),
        dtype=float
    )

    for i in range(n_satellites):
        multipath[:, i] = (
            amplitudes[i]
            * np.sin(omega * t + phases[i])
        )

    return multipath


def multipath_sinusoidal(t, n_satellites, amplitudes=None, omega=0.05):
    """
    Alias conservé pour compatibilité avec le code existant.

    Utiliser de préférence generate_multipath().
    """

    return generate_multipath(
        t=t,
        n_satellites=n_satellites,
        amplitudes=amplitudes,
        omega=omega
    )