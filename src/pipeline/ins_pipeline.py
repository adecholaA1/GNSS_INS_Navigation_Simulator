"""
ins_pipeline.py

Pipeline INS :
- simulation accéléromètre ;
- simulation gyroscope ;
- mécanisation Strapdown ;
- évaluation de deux scénarios IMU :
  1. IMU nominale ;
  2. IMU bruitée / biaisée.
"""

import numpy as np

from src.sensors.imu import simulate_accelerometer
from src.sensors.gyroscope import simulate_gyroscope
from src.ins.mechanization import run_strapdown_ins


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def run_ins_pipeline(
    trajectory,
    seed=42,
):
    """
    Exécute la navigation inertielle Strapdown pour deux scénarios IMU.
    """

    dt = trajectory["dt"]

    true_position = trajectory["position"]
    true_velocity = trajectory["velocity"]

    specific_force_body = trajectory["specific_force_body"]
    angular_rates = trajectory["angular_rates"]

    initial_position = trajectory["initial_position"]
    initial_velocity = trajectory["initial_velocity"]
    initial_attitude = trajectory["initial_attitude"]

    reference_attitudes = trajectory["quaternions"]

    imu_scenarios = {
        "nominal": {
            "label": "IMU nominale",
            "acc_sigma": 0.003,
            "acc_bias": np.array([0.0, 0.0, 0.0]),
            "gyro_sigma": 0.0001,
            "gyro_bias": np.array([0.0, 0.0, 0.0]),
        },
        "noisy": {
            "label": "IMU bruitée / biaisée",
            "acc_sigma": 0.03,
            "acc_bias": np.array([0.01, -0.01, 0.005]),
            "gyro_sigma": 0.001,
            "gyro_bias": np.array([0.0, 0.0, 0.0005]),
        },
    }

    results = {}

    for scenario_name, scenario in imu_scenarios.items():

        accelerometer_measurements = simulate_accelerometer(
            true_acceleration=specific_force_body,
            sigma=scenario["acc_sigma"],
            bias=scenario["acc_bias"],
            seed=seed,
        )

        gyroscope_measurements = simulate_gyroscope(
            true_angular_rate=angular_rates,
            sigma=scenario["gyro_sigma"],
            bias=scenario["gyro_bias"],
            seed=seed,
        )

        (
            strapdown_position,
            strapdown_velocity,
            strapdown_attitude,
        ) = run_strapdown_ins(
            initial_position=initial_position,
            initial_velocity=initial_velocity,
            initial_attitude=initial_attitude,
            accelerometer_measurements=accelerometer_measurements,
            gyroscope_measurements=gyroscope_measurements,
            dt=dt,
            reference_attitudes=reference_attitudes,
        )

        error = np.linalg.norm(
            strapdown_position - true_position,
            axis=1,
        )

        rmse = compute_rmse(
            error,
        )

        results[scenario_name] = {
            "label": scenario["label"],
            "accelerometer_measurements": accelerometer_measurements,
            "gyroscope_measurements": gyroscope_measurements,
            "position": strapdown_position,
            "velocity": strapdown_velocity,
            "attitude": strapdown_attitude,
            "error": error,
            "rmse": rmse,
            "acc_sigma": scenario["acc_sigma"],
            "acc_bias": scenario["acc_bias"],
            "gyro_sigma": scenario["gyro_sigma"],
            "gyro_bias": scenario["gyro_bias"],
        }

    return results