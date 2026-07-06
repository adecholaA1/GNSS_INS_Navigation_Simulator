"""
Solveur GNSS par méthode de Gauss-Newton.

Ce module estime la position d'un récepteur à partir de pseudodistances
GNSS et de positions satellites connues.

Le problème est non linéaire car les distances dépendent de la norme
euclidienne entre le récepteur et les satellites. La méthode de
Gauss-Newton linéarise ce problème autour d'une position courante,
puis corrige progressivement l'estimation.
"""

import numpy as np

from src.gnss.pseudorange import compute_distances


def compute_jacobian(receiver_estimate, satellites):
    """
    Calcule la matrice Jacobienne du modèle de distance GNSS.

    Chaque ligne contient la dérivée de la distance satellite-récepteur
    par rapport aux coordonnées du récepteur.

    Parameters
    ----------
    receiver_estimate : ndarray (3,)
        Position courante estimée du récepteur.

    satellites : ndarray (N, 3)
        Positions des satellites.

    Returns
    -------
    ndarray (N, 3)
        Matrice Jacobienne du problème GNSS.
    """

    jacobian = []

    for satellite in satellites:
        diff = receiver_estimate - satellite
        distance = np.linalg.norm(diff)

        if distance < 1e-12:
            distance = 1e-12

        jacobian.append(diff / distance)

    return np.array(jacobian)


def solve_position_gauss_newton(
    satellites,
    pseudoranges,
    initial_position,
    max_iterations=10,
    tolerance=1e-4
):
    """
    Estime la position du récepteur par Gauss-Newton.

    À chaque itération, le solveur :
    - prédit les distances à partir de la position courante ;
    - calcule les résidus entre distances prédites et pseudodistances ;
    - linéarise le modèle avec la Jacobienne ;
    - applique une correction de position.

    Parameters
    ----------
    satellites : ndarray (N, 3)
        Positions des satellites.

    pseudoranges : ndarray (N,)
        Pseudodistances mesurées.

    initial_position : ndarray (3,)
        Point de départ de l'algorithme.

    max_iterations : int
        Nombre maximal d'itérations.

    tolerance : float
        Seuil d'arrêt sur la norme de la correction.

    Returns
    -------
    position : ndarray (3,)
        Position estimée du récepteur.

    history : list of dict
        Historique de convergence.
    """

    position = initial_position.astype(float).copy()
    history = []

    for iteration in range(max_iterations):

        predicted_ranges = compute_distances(
            position,
            satellites
        )

        residuals = predicted_ranges - pseudoranges

        jacobian = compute_jacobian(
            position,
            satellites
        )

        correction = np.linalg.lstsq(
            jacobian,
            residuals,
            rcond=None
        )[0]

        position = position - correction

        history.append(
            {
                "iteration": iteration + 1,
                "residual_norm": np.linalg.norm(residuals),
                "step_norm": np.linalg.norm(correction),
            }
        )

        if np.linalg.norm(correction) < tolerance:
            break

    return position, history