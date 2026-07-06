"""
raim_pipeline.py

Pipeline RAIM/FDE :
- injection d'un défaut satellite ;
- estimation Gauss-Newton non protégée ;
- estimation protégée RAIM/FDE ;
- statistiques d'intégrité.
"""

import numpy as np

from src.gnss.gauss_newton import solve_position_gauss_newton
from src.gnss.raim import raim_fde
from src.signal_processing.noise import satellite_bias


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def run_raim_pipeline(
    trajectory,
    gnss,
    fault_satellite=2,
    fault_bias=30.0,
    sigma=2.0,
    pfa=1e-3,
):
    """
    Exécute RAIM/FDE sur les pseudodistances GNSS bruitées.
    """

    t = trajectory["t"]
    true_position = trajectory["position"]

    satellites = gnss["satellites"]
    pseudoranges_nominal = gnss["pseudoranges_noisy"]

    fault = satellite_bias(
        pseudoranges_nominal.shape,
        satellite_index=fault_satellite,
        bias=fault_bias,
    )

    pseudoranges_faulty = pseudoranges_nominal + fault

    estimated_raw = []
    estimated_raim = []

    raim_statistics = []
    raim_thresholds = []
    raim_flags = []
    excluded_satellites = []

    x0_raw = true_position[0] + np.array([50.0, -50.0, 20.0])
    x0_raim = x0_raw.copy()

    for k in range(len(t)):

        raw_position, _ = solve_position_gauss_newton(
            satellites=satellites,
            pseudoranges=pseudoranges_faulty[k],
            initial_position=x0_raw,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_raw.append(raw_position)
        x0_raw = raw_position

        raim_result = raim_fde(
            satellites=satellites,
            pseudoranges=pseudoranges_faulty[k],
            initial_position=x0_raim,
            sigma=sigma,
            pfa=pfa,
            max_iterations=30,
            tolerance=1e-4,
        )

        raim_position = raim_result["position"]

        estimated_raim.append(raim_position)
        x0_raim = raim_position

        raim_statistics.append(raim_result["statistic"])
        raim_thresholds.append(raim_result["threshold"])
        raim_flags.append(raim_result["fault_detected"])
        excluded_satellites.append(raim_result["excluded_satellite"])

    estimated_raw = np.array(estimated_raw)
    estimated_raim = np.array(estimated_raim)

    raim_statistics = np.array(raim_statistics)
    raim_thresholds = np.array(raim_thresholds)
    raim_flags = np.array(raim_flags, dtype=bool)

    error_raw = np.linalg.norm(
        estimated_raw - true_position,
        axis=1,
    )

    error_raim = np.linalg.norm(
        estimated_raim - true_position,
        axis=1,
    )

    rmse_raw = compute_rmse(error_raw)
    rmse_raim = compute_rmse(error_raim)

    valid_exclusions = [
        index for index in excluded_satellites
        if index is not None
    ]

    most_common_excluded = None

    if valid_exclusions:
        most_common_excluded = max(
            set(valid_exclusions),
            key=valid_exclusions.count,
        )

    return {
        "fault_satellite": fault_satellite,
        "fault_bias": fault_bias,
        "pseudoranges_faulty": pseudoranges_faulty,
        "estimated_raw": estimated_raw,
        "estimated_raim": estimated_raim,
        "error_raw": error_raw,
        "error_raim": error_raim,
        "rmse_raw": rmse_raw,
        "rmse_raim": rmse_raim,
        "statistics": raim_statistics,
        "thresholds": raim_thresholds,
        "flags": raim_flags,
        "n_detections": int(np.sum(raim_flags)),
        "excluded_satellites": excluded_satellites,
        "most_common_excluded": most_common_excluded,
        "pfa": pfa,
    }