import numpy as np


# ======================================================================
# Constantes physiques
# ======================================================================

# Vitesse de propagation du signal GNSS dans le vide (m/s)

SPEED_OF_LIGHT = 299_792_458.0


# ======================================================================
# Distance géométrique
# ======================================================================

def geometric_distance(receiver, satellite):
    """
    Calcule la distance géométrique entre un récepteur GNSS
    et un satellite.

    Cette grandeur correspond à la distance euclidienne idéale
    entre les deux positions, sans tenir compte des erreurs de
    mesure ni des effets de propagation.

    Parameters
    ----------
    receiver : ndarray (3,)
        Position du récepteur [x, y, z] en mètres.

    satellite : ndarray (3,)
        Position du satellite [x, y, z] en mètres.

    Returns
    -------
    float
        Distance géométrique en mètres.
    """

    return np.linalg.norm(satellite - receiver)


# ======================================================================
# Distances géométriques
# ======================================================================

def compute_distances(receiver, satellites):
    """
    Calcule les distances géométriques entre un récepteur
    et l'ensemble de la constellation GNSS.

    Cette fonction constitue le modèle géométrique utilisé
    avant l'ajout des différentes sources d'erreurs
    (horloge, bruit, multipath, etc.).

    Parameters
    ----------
    receiver : ndarray (3,)
        Position du récepteur.

    satellites : ndarray (N,3)
        Positions des satellites.

    Returns
    -------
    ndarray (N,)
        Distances géométriques en mètres.
    """

    return np.linalg.norm(
        satellites - receiver,
        axis=1
    )


# ======================================================================
# Pseudodistances GNSS
# ======================================================================

def compute_pseudoranges(
    receiver,
    satellites,
    clock_bias_seconds=0.0,
    noise=None
):
    """
    Génère les pseudodistances GNSS simulées.

    Le modèle utilisé est :

        ρ = d + c·Δt + ε

    où :

        d   : distance géométrique
        c   : vitesse de la lumière
        Δt  : biais d'horloge du récepteur
        ε   : erreurs de mesure (bruit, multipath, etc.)

    Cette fonction représente le modèle de mesure GNSS
    utilisé par les algorithmes de navigation.

    Parameters
    ----------
    receiver : ndarray (3,)
        Position du récepteur.

    satellites : ndarray (N,3)
        Positions des satellites.

    clock_bias_seconds : float, optional
        Biais d'horloge du récepteur (s).

    noise : ndarray (N,), optional
        Erreurs de mesure exprimées en mètres.

    Notes
    -----
    Cette fonction implémente uniquement le modèle nominal des
    pseudodistances. Les différentes erreurs de mesure sont ajoutées
    séparément afin de faciliter leur étude et leur évolution.

    Returns
    -------
    ndarray (N,)
        Pseudodistances simulées en mètres.
    """

    # Distances géométriques

    distances = compute_distances(
        receiver,
        satellites
    )

    # Conversion du biais d'horloge en mètres

    clock_bias_meters = (
        SPEED_OF_LIGHT
        * clock_bias_seconds
    )

    # ------------------------------------------------------------------
    # Modèle de mesure GNSS
    #
    # Cette première implémentation considère uniquement :
    #   • la distance géométrique ;
    #   • le biais d'horloge du récepteur.
    #
    # Les autres sources d'erreurs (bruit, multipath, biais satellites,
    # jamming, spoofing, délais ionosphériques et troposphériques, etc.)
    # sont volontairement générées dans des modules indépendants afin
    # de conserver une architecture modulaire et facilement extensible.
    # ------------------------------------------------------------------

    pseudoranges = (
        distances
        + clock_bias_meters
    )

    # Ajout des erreurs de mesure

    if noise is not None:

        pseudoranges += noise

    return pseudoranges