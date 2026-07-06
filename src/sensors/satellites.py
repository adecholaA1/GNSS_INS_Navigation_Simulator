import numpy as np


def generate_satellites():
    """
    Génère une constellation GNSS simplifiée.

    Les positions sont exprimées dans un repère cartésien 3D assimilé
    au repère ECEF, en mètres.

    Cette constellation fixe permet de tester les algorithmes GNSS
    sans introduire immédiatement la dynamique orbitale des satellites.

    Returns
    -------
    satellites : ndarray (N, 3)
        Positions des satellites [x, y, z] en mètres.
    """

    satellites = np.array(
        [
            [15600000.0,  7540000.0, 20140000.0],
            [18760000.0,  2750000.0, 18610000.0],
            [17610000.0, 14630000.0, 13480000.0],
            [19170000.0,   610000.0, 18390000.0],
            [17800000.0, -8200000.0, 21000000.0],
            [20200000.0,  4200000.0, 17600000.0],
        ],
        dtype=float
    )

    return satellites