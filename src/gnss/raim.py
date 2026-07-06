"""
RAIM GNSS : Receiver Autonomous Integrity Monitoring.

Ce module implémente une chaîne RAIM proche d'une architecture
industrielle :

- estimation de position par moindres carrés ;
- calcul des résidus de pseudodistances ;
- test global d'intégrité par statistique χ² ;
- Fault Detection and Exclusion (FDE) ;
- recherche du meilleur sous-ensemble de satellites.

Le modèle estime uniquement la position 3D.
Le biais d'horloge est supposé déjà compensé dans ce simulateur.
"""

import numpy as np
from scipy.stats import chi2

from src.gnss.gauss_newton import solve_position_gauss_newton
from src.gnss.pseudorange import compute_distances


def compute_pseudorange_residuals(
    receiver_position,
    satellites,
    pseudoranges,
):
    """
    Calcule les résidus de pseudodistance.

        r = rho_mesuree - rho_predite
    """

    predicted_ranges = compute_distances(
        receiver_position,
        satellites,
    )

    return pseudoranges - predicted_ranges


def compute_raim_statistic(
    residuals,
    sigma,
):
    """
    Statistique RAIM :

        T = rᵀ R⁻¹ r

    avec

        R = sigma² I
    """

    n = len(residuals)

    R = np.eye(n) * sigma**2

    W = np.linalg.inv(R)

    statistic = residuals.T @ W @ residuals

    return statistic


def global_raim_test(
    residuals,
    sigma,
    pfa=1e-3,
):
    """
    Test global RAIM basé sur la loi du χ².

    Parameters
    ----------
    residuals : ndarray
        Résidus de pseudodistance.

    sigma : float
        Ecart-type du bruit.

    pfa : float
        Probabilité de fausse alarme.

    Returns
    -------
    passed : bool
        Test réussi.

    statistic : float
        Statistique RAIM.

    threshold : float
        Seuil χ² utilisé.
    """

    statistic = compute_raim_statistic(
        residuals,
        sigma,
    )

    dof = len(residuals) - 4

    if dof <= 0:
        raise ValueError(
            "Nombre insuffisant de satellites pour effectuer un test RAIM."
        )

    threshold = chi2.ppf(
        1.0 - pfa,
        dof,
    )

    passed = statistic <= threshold

    return (
        passed,
        statistic,
        threshold,
    )


def evaluate_subset_solution(
    satellites,
    pseudoranges,
    initial_position,
    sigma,
    max_iterations=30,
    tolerance=1e-4,
):
    """
    Estime une position et calcule la statistique RAIM associée.
    """

    estimated_position, history = solve_position_gauss_newton(
        satellites=satellites,
        pseudoranges=pseudoranges,
        initial_position=initial_position,
        max_iterations=max_iterations,
        tolerance=tolerance,
    )

    residuals = compute_pseudorange_residuals(
        receiver_position=estimated_position,
        satellites=satellites,
        pseudoranges=pseudoranges,
    )

    statistic = compute_raim_statistic(
        residuals,
        sigma,
    )

    return {
        "position": estimated_position,
        "residuals": residuals,
        "statistic": statistic,
        "history": history,
    }


def raim_fde(
    satellites,
    pseudoranges,
    initial_position,
    sigma=2.0,
    pfa=1e-3,
    max_iterations=30,
    tolerance=1e-4,
):
    """
    RAIM avec Fault Detection and Exclusion (FDE).

    Parameters
    ----------
    satellites : ndarray (N,3)

    pseudoranges : ndarray (N,)

    initial_position : ndarray (3,)

    sigma : float
        Ecart-type des pseudodistances.

    pfa : float
        Probabilité de fausse alarme.

    Returns
    -------
    dict
    """

    n_satellites = satellites.shape[0]

    full_solution = evaluate_subset_solution(
        satellites=satellites,
        pseudoranges=pseudoranges,
        initial_position=initial_position,
        sigma=sigma,
        max_iterations=max_iterations,
        tolerance=tolerance,
    )

    passed, statistic, threshold = global_raim_test(
        residuals=full_solution["residuals"],
        sigma=sigma,
        pfa=pfa,
    )

    if passed:

        return {
            "position": full_solution["position"],
            "raim_ok": True,
            "fault_detected": False,
            "excluded_satellite": None,
            "statistic": statistic,
            "threshold": threshold,
            "residuals": full_solution["residuals"],
            "used_satellites": np.arange(n_satellites),
        }

    best_solution = None
    best_statistic = np.inf
    best_excluded = None
    best_used = None

    for excluded_index in range(n_satellites):

        used_indices = np.delete(
            np.arange(n_satellites),
            excluded_index,
        )

        subset_solution = evaluate_subset_solution(
            satellites=satellites[used_indices],
            pseudoranges=pseudoranges[used_indices],
            initial_position=initial_position,
            sigma=sigma,
            max_iterations=max_iterations,
            tolerance=tolerance,
        )

        if subset_solution["statistic"] < best_statistic:

            best_statistic = subset_solution["statistic"]
            best_solution = subset_solution
            best_excluded = excluded_index
            best_used = used_indices

    _, _, chi2_threshold = global_raim_test(
        residuals=best_solution["residuals"],
        sigma=sigma,
        pfa=pfa,
    )

    raim_ok_after_exclusion = (
        best_statistic <= chi2_threshold
    )

    return {
        "position": best_solution["position"],
        "raim_ok": raim_ok_after_exclusion,
        "fault_detected": True,
        "excluded_satellite": best_excluded,
        "statistic": best_statistic,
        "threshold": chi2_threshold,
        "residuals": best_solution["residuals"],
        "used_satellites": best_used,
    }




















# """
# RAIM GNSS : Receiver Autonomous Integrity Monitoring.

# Ce module implémente une chaîne RAIM exploitable dans un simulateur
# GNSS industriel :

# - estimation de position par moindres carrés ;
# - calcul des résidus de pseudodistances ;
# - test global d'intégrité ;
# - recherche d'un satellite fautif par exclusion ;
# - recalcul de la solution avec le meilleur sous-ensemble.

# Le modèle estime uniquement la position 3D du récepteur.
# Le biais d'horloge est supposé déjà compensé dans ce simulateur.
# """

# import numpy as np

# from src.gnss.gauss_newton import solve_position_gauss_newton
# from src.gnss.pseudorange import compute_distances


# def compute_pseudorange_residuals(
#     receiver_position,
#     satellites,
#     pseudoranges,
# ):
#     """
#     Calcule les résidus de pseudodistance.

#     Résidu :
#         r_i = rho_mesuree_i - rho_predite_i
#     """

#     predicted_ranges = compute_distances(
#         receiver_position,
#         satellites,
#     )

#     return pseudoranges - predicted_ranges


# def compute_raim_statistic(residuals, sigma):
#     """
#     Calcule la statistique RAIM globale.

#     La statistique utilisée est la somme normalisée des carrés
#     des résidus :

#         T = sum((r_i / sigma)^2)
#     """

#     return np.sum((residuals / sigma) ** 2)


# def global_raim_test(
#     residuals,
#     sigma,
#     threshold,
# ):
#     """
#     Test global d'intégrité RAIM.
#     """

#     statistic = compute_raim_statistic(
#         residuals=residuals,
#         sigma=sigma,
#     )

#     passed = statistic <= threshold

#     return passed, statistic


# def evaluate_subset_solution(
#     satellites,
#     pseudoranges,
#     initial_position,
#     sigma,
#     max_iterations=30,
#     tolerance=1e-4,
# ):
#     """
#     Estime une position et calcule la statistique RAIM associée.
#     """

#     estimated_position, history = solve_position_gauss_newton(
#         satellites=satellites,
#         pseudoranges=pseudoranges,
#         initial_position=initial_position,
#         max_iterations=max_iterations,
#         tolerance=tolerance,
#     )

#     residuals = compute_pseudorange_residuals(
#         receiver_position=estimated_position,
#         satellites=satellites,
#         pseudoranges=pseudoranges,
#     )

#     statistic = compute_raim_statistic(
#         residuals=residuals,
#         sigma=sigma,
#     )

#     return {
#         "position": estimated_position,
#         "residuals": residuals,
#         "statistic": statistic,
#         "history": history,
#     }


# def raim_fde(
#     satellites,
#     pseudoranges,
#     initial_position,
#     sigma=2.0,
#     threshold=25.0,
#     max_iterations=30,
#     tolerance=1e-4,
# ):
#     """
#     RAIM avec Fault Detection and Exclusion.

#     Parameters
#     ----------
#     satellites : ndarray (N,3)
#         Positions satellites.

#     pseudoranges : ndarray (N,)
#         Pseudodistances mesurées.

#     initial_position : ndarray (3,)
#         Initialisation Gauss-Newton.

#     sigma : float
#         Ecart-type attendu du bruit pseudodistance.

#     threshold : float
#         Seuil RAIM global.

#     Returns
#     -------
#     dict
#         Résultat RAIM complet.
#     """

#     n_satellites = satellites.shape[0]

#     full_solution = evaluate_subset_solution(
#         satellites=satellites,
#         pseudoranges=pseudoranges,
#         initial_position=initial_position,
#         sigma=sigma,
#         max_iterations=max_iterations,
#         tolerance=tolerance,
#     )

#     passed, statistic = global_raim_test(
#         residuals=full_solution["residuals"],
#         sigma=sigma,
#         threshold=threshold,
#     )

#     if passed:
#         return {
#             "position": full_solution["position"],
#             "raim_ok": True,
#             "fault_detected": False,
#             "excluded_satellite": None,
#             "statistic": statistic,
#             "residuals": full_solution["residuals"],
#             "used_satellites": np.arange(n_satellites),
#         }

#     best_solution = None
#     best_statistic = np.inf
#     best_excluded = None
#     best_used = None

#     for excluded_index in range(n_satellites):

#         used_indices = np.array([
#             i for i in range(n_satellites)
#             if i != excluded_index
#         ])

#         subset_satellites = satellites[used_indices]
#         subset_pseudoranges = pseudoranges[used_indices]

#         subset_solution = evaluate_subset_solution(
#             satellites=subset_satellites,
#             pseudoranges=subset_pseudoranges,
#             initial_position=initial_position,
#             sigma=sigma,
#             max_iterations=max_iterations,
#             tolerance=tolerance,
#         )

#         if subset_solution["statistic"] < best_statistic:
#             best_statistic = subset_solution["statistic"]
#             best_solution = subset_solution
#             best_excluded = excluded_index
#             best_used = used_indices

#     raim_ok_after_exclusion = best_statistic <= threshold

#     return {
#         "position": best_solution["position"],
#         "raim_ok": raim_ok_after_exclusion,
#         "fault_detected": True,
#         "excluded_satellite": best_excluded,
#         "statistic": best_statistic,
#         "residuals": best_solution["residuals"],
#         "used_satellites": best_used,
#     }