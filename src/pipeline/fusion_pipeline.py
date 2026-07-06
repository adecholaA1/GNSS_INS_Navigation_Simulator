"""
fusion_pipeline.py

Pipeline de fusion GNSS/INS avec GNSSINSKalman.
"""

import numpy as np

from src.fusion.fusion_kalman import GNSSINSKalman


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def run_fusion_pipeline(
    trajectory,
    kalman,
    ins,
):
    true_position = trajectory["position"]
    true_velocity = trajectory["velocity"]

    gnss_position = kalman["estimated_positions"]

    results = {}

    for scenario_name, scenario in ins.items():

        fusion_filter = GNSSINSKalman()

        fusion_filter.initialize(
            position=true_position[0],
            velocity=true_velocity[0],
        )

        fusion_position = np.zeros_like(true_position)
        fusion_velocity = np.zeros_like(true_velocity)

        for k in range(len(true_position)):

            fusion_filter.predict(
                scenario["position"][k],
                scenario["velocity"][k],
            )

            fusion_filter.update(
                gnss_position[k],
            )

            state = fusion_filter.state()

            fusion_position[k] = state[:3]
            fusion_velocity[k] = state[3:]

        error = np.linalg.norm(
            fusion_position - true_position,
            axis=1,
        )

        rmse = compute_rmse(error)

        results[scenario_name] = {
            "label": scenario["label"],
            "position": fusion_position,
            "velocity": fusion_velocity,
            "error": error,
            "rmse": rmse,
        }

    return results