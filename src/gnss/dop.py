"""
Calcul des indicateurs DOP GNSS.

Les DOP (Dilution of Precision) caractérisent la qualité géométrique
de la constellation GNSS vue par le récepteur.

Plus les valeurs sont faibles, meilleure est la géométrie satellite.
"""

import numpy as np


def compute_geometry_matrix(receiver_position, satellites):
    """
    Construit la matrice géométrique GNSS.

    Chaque ligne contient le vecteur unitaire récepteur -> satellite
    et une colonne associée au biais d'horloge.
    """

    H = []

    for satellite in satellites:
        line_of_sight = satellite - receiver_position
        distance = np.linalg.norm(line_of_sight)

        if distance < 1e-12:
            distance = 1e-12

        unit_vector = line_of_sight / distance

        H.append([
            unit_vector[0],
            unit_vector[1],
            unit_vector[2],
            1.0,
        ])

    return np.array(H, dtype=float)


def compute_dop(receiver_position, satellites):
    """
    Calcule GDOP, PDOP, HDOP, VDOP et TDOP.

    Parameters
    ----------
    receiver_position : ndarray (3,)
        Position du récepteur.

    satellites : ndarray (N,3)
        Positions des satellites.

    Returns
    -------
    dict
        Indicateurs de dilution de précision.
    """

    H = compute_geometry_matrix(receiver_position, satellites)

    Q = np.linalg.inv(H.T @ H)

    gdop = np.sqrt(Q[0, 0] + Q[1, 1] + Q[2, 2] + Q[3, 3])
    pdop = np.sqrt(Q[0, 0] + Q[1, 1] + Q[2, 2])
    hdop = np.sqrt(Q[0, 0] + Q[1, 1])
    vdop = np.sqrt(Q[2, 2])
    tdop = np.sqrt(Q[3, 3])

    return {
        "GDOP": gdop,
        "PDOP": pdop,
        "HDOP": hdop,
        "VDOP": vdop,
        "TDOP": tdop,
    }