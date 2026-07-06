"""
Génération de constellations GNSS.

Ce module permet de générer des constellations GPS, Galileo,
GLONASS ou BeiDou de manière paramétrable.

Les satellites sont générés sur une sphère de rayon orbital.
Cette version constitue une approximation géométrique adaptée
aux simulations algorithmiques GNSS.
"""

import numpy as np


GPS_ORBIT_RADIUS = 26_560_000.0
GALILEO_ORBIT_RADIUS = 29_600_000.0
GLONASS_ORBIT_RADIUS = 25_510_000.0
BEIDOU_ORBIT_RADIUS = 27_900_000.0


def _generate_constellation(
    n_satellites,
    orbit_radius,
    seed=None,
):
    """
    Génère une constellation uniformément répartie
    sur une sphère.

    Parameters
    ----------
    n_satellites : int
        Nombre de satellites.

    orbit_radius : float
        Rayon orbital (m).

    seed : int | None
        Graine aléatoire.

    Returns
    -------
    ndarray (N,3)
        Coordonnées ECEF des satellites.
    """

    rng = np.random.default_rng(seed)

    satellites = np.zeros((n_satellites, 3))

    golden_angle = np.pi * (3.0 - np.sqrt(5.0))

    for i in range(n_satellites):

        z = 1.0 - (2.0 * i + 1.0) / n_satellites

        radius_xy = np.sqrt(1.0 - z**2)

        theta = golden_angle * i

        x = radius_xy * np.cos(theta)
        y = radius_xy * np.sin(theta)

        direction = np.array([x, y, z])

        # petite perturbation aléatoire
        direction += rng.normal(0.0, 0.015, 3)

        direction /= np.linalg.norm(direction)

        satellites[i] = orbit_radius * direction

    return satellites


def generate_gps_constellation(
    n_satellites=12,
    seed=None,
):
    """
    Génère une constellation GPS.
    """

    return _generate_constellation(
        n_satellites=n_satellites,
        orbit_radius=GPS_ORBIT_RADIUS,
        seed=seed,
    )


def generate_galileo_constellation(
    n_satellites=12,
    seed=None,
):
    """
    Génère une constellation Galileo.
    """

    return _generate_constellation(
        n_satellites=n_satellites,
        orbit_radius=GALILEO_ORBIT_RADIUS,
        seed=seed,
    )


def generate_glonass_constellation(
    n_satellites=12,
    seed=None,
):
    """
    Génère une constellation GLONASS.
    """

    return _generate_constellation(
        n_satellites=n_satellites,
        orbit_radius=GLONASS_ORBIT_RADIUS,
        seed=seed,
    )


def generate_beidou_constellation(
    n_satellites=12,
    seed=None,
):
    """
    Génère une constellation BeiDou.
    """

    return _generate_constellation(
        n_satellites=n_satellites,
        orbit_radius=BEIDOU_ORBIT_RADIUS,
        seed=seed,
    )


def generate_multi_constellation(
    gps=8,
    galileo=6,
    glonass=0,
    beidou=0,
    seed=42,
):
    """
    Génère une constellation multi-GNSS.

    Returns
    -------
    ndarray (N,3)
    """

    satellites = []

    if gps > 0:
        satellites.append(
            generate_gps_constellation(
                gps,
                seed=seed,
            )
        )

    if galileo > 0:
        satellites.append(
            generate_galileo_constellation(
                galileo,
                seed=seed + 1,
            )
        )

    if glonass > 0:
        satellites.append(
            generate_glonass_constellation(
                glonass,
                seed=seed + 2,
            )
        )

    if beidou > 0:
        satellites.append(
            generate_beidou_constellation(
                beidou,
                seed=seed + 3,
            )
        )

    return np.vstack(satellites)