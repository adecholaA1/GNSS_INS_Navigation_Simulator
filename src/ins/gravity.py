"""
Modèle de gravité pour la navigation inertielle.

Dans cette première version, la gravité est supposée constante et orientée
selon l'axe vertical du repère de navigation.
"""

import numpy as np


GRAVITY_MAGNITUDE = 9.80665  # m/s²


def gravity_vector():
    """
    Retourne le vecteur gravité dans le repère de navigation.

    Convention utilisée :
        z vers le haut

    Returns
    -------
    ndarray (3,)
        Vecteur gravité [gx, gy, gz] en m/s².
    """

    return np.array([0.0, 0.0, -GRAVITY_MAGNITUDE])