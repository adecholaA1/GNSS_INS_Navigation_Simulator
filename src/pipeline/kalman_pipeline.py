"""
kalman_pipeline.py

Pipeline Kalman GNSS :
- prend la solution GNSS protégée RAIM ;
- applique un filtre de Kalman 3D position/vitesse ;
- calcule l'erreur et la RMSE.
"""

import numpy as np

from src.fusion.kalman import KalmanFilter3D


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def run_kalman_pipeline(
    trajectory,
    raim,
    process_noise=0.1,
    measurement_noise=25.0,
):
    """
    Filtre la solution GNSS protégée RAIM avec un Kalman 3D.
    """

    dt = trajectory["dt"]
    true_position = trajectory["position"]

    gnss_measurements = raim["estimated_raim"]

    kalman = KalmanFilter3D(
        dt=dt,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
    )

    kalman.initialize(
        gnss_measurements[0],
    )

    estimated_positions = []

    for measurement in gnss_measurements:

        kalman.predict()

        position_kalman = kalman.update(
            measurement,
        )

        estimated_positions.append(
            position_kalman,
        )

    estimated_positions = np.array(
        estimated_positions,
    )

    error = np.linalg.norm(
        estimated_positions - true_position,
        axis=1,
    )

    rmse = compute_rmse(
        error,
    )

    return {
        "estimated_positions": estimated_positions,
        "error": error,
        "rmse": rmse,
        "process_noise": process_noise,
        "measurement_noise": measurement_noise,
    }