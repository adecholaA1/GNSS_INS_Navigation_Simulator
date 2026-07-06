"""
gnss_pipeline.py

Pipeline GNSS nominal :
- génération constellation ;
- calcul des pseudodistances vraies ;
- ajout bruit blanc + multipath ;
- estimation Gauss-Newton nominale ;
- calcul des DOP.
"""

import numpy as np

from src.gnss.constellation_generator import generate_multi_constellation
from src.gnss.pseudorange import compute_pseudoranges
from src.gnss.gauss_newton import solve_position_gauss_newton
from src.gnss.dop import compute_dop
from src.signal_processing.noise import gaussian_noise, multipath_sinusoidal


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def run_gnss_pipeline(
    trajectory,
    gps=8,
    galileo=6,
    glonass=0,
    beidou=0,
    seed=42,
    pseudorange_sigma=2.0,
    multipath_omega=0.05,
):
    """
    Exécute la chaîne GNSS nominale sans défaut satellite.
    """

    t = trajectory["t"]
    position = trajectory["position"]

    satellites = generate_multi_constellation(
        gps=gps,
        galileo=galileo,
        glonass=glonass,
        beidou=beidou,
        seed=seed,
    )

    pseudoranges_true = np.array([
        compute_pseudoranges(
            receiver_position,
            satellites,
            clock_bias_seconds=0.0,
        )
        for receiver_position in position
    ])

    noise_gaussian = gaussian_noise(
        pseudoranges_true.shape,
        sigma=pseudorange_sigma,
        seed=seed,
    )

    noise_multipath = multipath_sinusoidal(
        t,
        n_satellites=satellites.shape[0],
        omega=multipath_omega,
    )

    pseudoranges_noisy = (
        pseudoranges_true
        + noise_gaussian
        + noise_multipath
    )

    estimated_positions = []

    x0 = position[0] + np.array([50.0, -50.0, 20.0])

    for k in range(len(t)):
        estimated_position, _ = solve_position_gauss_newton(
            satellites=satellites,
            pseudoranges=pseudoranges_noisy[k],
            initial_position=x0,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_positions.append(estimated_position)
        x0 = estimated_position

    estimated_positions = np.array(estimated_positions)

    error = np.linalg.norm(
        estimated_positions - position,
        axis=1,
    )

    rmse = compute_rmse(error)

    dop_history = {
        "GDOP": [],
        "PDOP": [],
        "HDOP": [],
        "VDOP": [],
        "TDOP": [],
    }

    for receiver_position in position:
        dop = compute_dop(
            receiver_position=receiver_position,
            satellites=satellites,
        )

        for key in dop_history:
            dop_history[key].append(dop[key])

    for key in dop_history:
        dop_history[key] = np.array(dop_history[key])

    dop_first = {
        key: values[0]
        for key, values in dop_history.items()
    }

    dop_mean = {
        key: float(np.mean(values))
        for key, values in dop_history.items()
    }

    return {
        "configuration": "GPS + Galileo",
        "satellites": satellites,
        "pseudoranges_true": pseudoranges_true,
        "pseudoranges_noisy": pseudoranges_noisy,
        "noise_gaussian": noise_gaussian,
        "noise_multipath": noise_multipath,
        "estimated_positions": estimated_positions,
        "error": error,
        "rmse": rmse,
        "dop_first": dop_first,
        "dop_mean": dop_mean,
        "dop_history": dop_history,
        "pseudorange_sigma": pseudorange_sigma,
    }