import numpy as np
import matplotlib.pyplot as plt

from src.simulation.trajectory import generate_drone_trajectory
from src.gnss.constellation_generator import generate_multi_constellation
from src.gnss.pseudorange import compute_pseudoranges
from src.gnss.gauss_newton import solve_position_gauss_newton
from src.gnss.raim import raim_fde
from src.gnss.fault_injection import inject_time_window_bias
from src.signal_processing.noise import (
    gaussian_noise,
    multipath_sinusoidal,
)


def compute_rmse(error):
    return np.sqrt(np.mean(error ** 2))


def main():

    dt = 0.1

    PFA = 1e-3

    fault_satellite = 2
    fault_start = 60.0
    fault_end = 120.0
    fault_bias = 30.0

    t, position, _, _ = generate_drone_trajectory(
        duration=240.0,
        dt=dt,
    )

    satellites = generate_multi_constellation(
        gps=8,
        galileo=6,
        glonass=0,
        beidou=0,
        seed=42,
    )

    pseudoranges = np.array([
        compute_pseudoranges(
            receiver,
            satellites,
            clock_bias_seconds=0.0,
        )
        for receiver in position
    ])

    pseudoranges_nominal = (
        pseudoranges
        + gaussian_noise(
            pseudoranges.shape,
            sigma=2.0,
            seed=42,
        )
        + multipath_sinusoidal(
            t,
            n_satellites=satellites.shape[0],
            omega=0.05,
        )
    )

    pseudoranges_faulty = inject_time_window_bias(
        pseudoranges=pseudoranges_nominal,
        satellite_index=fault_satellite,
        start_time=fault_start,
        end_time=fault_end,
        bias=fault_bias,
        dt=dt,
    )

    estimated_raw = []
    estimated_raim = []

    raim_statistics = []
    raim_fault_flags = []
    excluded_satellites = []

    initial_raw = position[0] + np.array([50.0, -50.0, 20.0])
    initial_raim = initial_raw.copy()

    raim_threshold = None

    for k in range(len(t)):

        raw_position, _ = solve_position_gauss_newton(
            satellites=satellites,
            pseudoranges=pseudoranges_faulty[k],
            initial_position=initial_raw,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_raw.append(raw_position)
        initial_raw = raw_position

        raim_result = raim_fde(
            satellites=satellites,
            pseudoranges=pseudoranges_faulty[k],
            initial_position=initial_raim,
            sigma=2.0,
            pfa=PFA,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_raim.append(
            raim_result["position"]
        )

        initial_raim = raim_result["position"]

        raim_statistics.append(
            raim_result["statistic"]
        )

        raim_fault_flags.append(
            raim_result["fault_detected"]
        )

        excluded_satellites.append(
            raim_result["excluded_satellite"]
        )

        raim_threshold = raim_result["threshold"]

    estimated_raw = np.array(estimated_raw)
    estimated_raim = np.array(estimated_raim)

    raim_statistics = np.array(raim_statistics)
    raim_fault_flags = np.array(
        raim_fault_flags,
        dtype=bool,
    )

    error_raw = np.linalg.norm(
        estimated_raw - position,
        axis=1,
    )

    error_raim = np.linalg.norm(
        estimated_raim - position,
        axis=1,
    )

    rmse_raw = compute_rmse(error_raw)
    rmse_raim = compute_rmse(error_raim)

    fault_mask = (
        (t >= fault_start)
        & (t <= fault_end)
    )

    rmse_raw_fault = compute_rmse(
        error_raw[fault_mask]
    )

    rmse_raim_fault = compute_rmse(
        error_raim[fault_mask]
    )

    detections_total = int(
        np.sum(raim_fault_flags)
    )

    detections_fault_window = int(
        np.sum(raim_fault_flags[fault_mask])
    )

    valid_exclusions = [
        index
        for index in excluded_satellites
        if index is not None
    ]

    print()
    print("=" * 60)
    print("Benchmark RAIM/FDE (Statistique χ²)")
    print("=" * 60)

    print(f"Nombre de satellites              : {satellites.shape[0]}")
    print(f"Satellite fautif simulé           : {fault_satellite + 1}")
    print(f"Fenêtre défaut                    : {fault_start:.1f} s -> {fault_end:.1f} s")
    print(f"Biais injecté                     : {fault_bias:.1f} m")
    print(f"PFA cible                         : {PFA:.1e}")
    print(f"Seuil χ²                          : {raim_threshold:.2f}")
    print()

    print(f"RMSE sans RAIM                    : {rmse_raw:.3f} m")
    print(f"RMSE avec RAIM                    : {rmse_raim:.3f} m")
    print(f"RMSE sans RAIM pendant défaut     : {rmse_raw_fault:.3f} m")
    print(f"RMSE avec RAIM pendant défaut     : {rmse_raim_fault:.3f} m")
    print()

    print(f"Détections RAIM totales           : {detections_total} / {len(t)}")

    print(
        f"Détections pendant défaut         : "
        f"{detections_fault_window} / {int(np.sum(fault_mask))}"
    )

    if valid_exclusions:

        most_common = max(
            set(valid_exclusions),
            key=valid_exclusions.count,
        )

        print(
            f"Satellite le plus souvent exclu   : "
            f"{most_common + 1}"
        )

    plt.figure(figsize=(12, 5))

    plt.plot(
        t,
        error_raw,
        label="Sans RAIM",
        alpha=0.7,
    )

    plt.plot(
        t,
        error_raim,
        label="Avec RAIM/FDE",
        linewidth=2,
    )

    plt.axvspan(
        fault_start,
        fault_end,
        alpha=0.2,
        label="Fenêtre défaut",
    )

    plt.title("Impact du RAIM/FDE")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12, 4))

    plt.plot(
        t,
        raim_statistics,
        label="Statistique χ²",
    )

    plt.axhline(
        raim_threshold,
        color="red",
        linestyle="--",
        label=f"Seuil χ² = {raim_threshold:.2f}",
    )

    plt.axvspan(
        fault_start,
        fault_end,
        alpha=0.2,
    )

    plt.title("Statistique globale RAIM")
    plt.xlabel("Temps (s)")
    plt.ylabel("Statistique")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(12, 4))

    plt.step(
        t,
        raim_fault_flags.astype(int),
        where="post",
    )

    plt.axvspan(
        fault_start,
        fault_end,
        alpha=0.2,
    )

    plt.title("Décision RAIM")
    plt.xlabel("Temps (s)")
    plt.ylabel("Détection")

    plt.yticks(
        [0, 1],
        ["Non", "Oui"],
    )

    plt.grid(True)
    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()























# import numpy as np
# import matplotlib.pyplot as plt

# from src.simulation.trajectory import generate_drone_trajectory
# from src.gnss.constellation_generator import generate_multi_constellation
# from src.gnss.pseudorange import compute_pseudoranges
# from src.gnss.gauss_newton import solve_position_gauss_newton
# from src.gnss.raim import raim_fde
# from src.gnss.fault_injection import inject_time_window_bias
# from src.signal_processing.noise import gaussian_noise, multipath_sinusoidal


# def compute_rmse(error):
#     return np.sqrt(np.mean(error**2))


# def main():
#     dt = 0.1

#     fault_satellite = 2
#     fault_start = 60.0
#     fault_end = 120.0
#     fault_bias = 30.0

#     t, position, _, _ = generate_drone_trajectory(
#         duration=240.0,
#         dt=dt,
#     )

#     satellites = generate_multi_constellation(
#         gps=8,
#         galileo=6,
#         glonass=0,
#         beidou=0,
#         seed=42,
#     )

#     pseudoranges = np.array([
#         compute_pseudoranges(
#             receiver,
#             satellites,
#             clock_bias_seconds=0.0,
#         )
#         for receiver in position
#     ])

#     pseudoranges_nominal = (
#         pseudoranges
#         + gaussian_noise(pseudoranges.shape, sigma=2.0, seed=42)
#         + multipath_sinusoidal(
#             t,
#             n_satellites=satellites.shape[0],
#             omega=0.05,
#         )
#     )

#     pseudoranges_faulty = inject_time_window_bias(
#         pseudoranges=pseudoranges_nominal,
#         satellite_index=fault_satellite,
#         start_time=fault_start,
#         end_time=fault_end,
#         bias=fault_bias,
#         dt=dt,
#     )

#     estimated_raw = []
#     estimated_raim = []

#     raim_statistics = []
#     raim_fault_flags = []
#     excluded_satellites = []

#     initial_raw = position[0] + np.array([50.0, -50.0, 20.0])
#     initial_raim = initial_raw.copy()

#     for k in range(len(t)):
#         raw_position, _ = solve_position_gauss_newton(
#             satellites=satellites,
#             pseudoranges=pseudoranges_faulty[k],
#             initial_position=initial_raw,
#             max_iterations=30,
#             tolerance=1e-4,
#         )

#         estimated_raw.append(raw_position)
#         initial_raw = raw_position

#         raim_result = raim_fde(
#             satellites=satellites,
#             pseudoranges=pseudoranges_faulty[k],
#             initial_position=initial_raim,
#             sigma=2.0,
#             threshold=25.0,
#             max_iterations=30,
#             tolerance=1e-4,
#         )

#         estimated_raim.append(raim_result["position"])
#         initial_raim = raim_result["position"]

#         raim_statistics.append(raim_result["statistic"])
#         raim_fault_flags.append(raim_result["fault_detected"])
#         excluded_satellites.append(raim_result["excluded_satellite"])

#     estimated_raw = np.array(estimated_raw)
#     estimated_raim = np.array(estimated_raim)
#     raim_statistics = np.array(raim_statistics)
#     raim_fault_flags = np.array(raim_fault_flags, dtype=bool)

#     error_raw = np.linalg.norm(estimated_raw - position, axis=1)
#     error_raim = np.linalg.norm(estimated_raim - position, axis=1)

#     rmse_raw = compute_rmse(error_raw)
#     rmse_raim = compute_rmse(error_raim)

#     fault_mask = (t >= fault_start) & (t <= fault_end)

#     rmse_raw_fault = compute_rmse(error_raw[fault_mask])
#     rmse_raim_fault = compute_rmse(error_raim[fault_mask])

#     detections_total = int(np.sum(raim_fault_flags))
#     detections_fault_window = int(np.sum(raim_fault_flags[fault_mask]))

#     valid_exclusions = [
#         index for index in excluded_satellites
#         if index is not None
#     ]

#     print("\n================ Benchmark RAIM/FDE ================")
#     print(f"Nombre de satellites              : {satellites.shape[0]}")
#     print(f"Satellite fautif simulé           : {fault_satellite + 1}")
#     print(f"Fenêtre défaut                    : {fault_start:.1f} s -> {fault_end:.1f} s")
#     print(f"Biais injecté                     : {fault_bias:.1f} m")
#     print()
#     print(f"RMSE sans RAIM                    : {rmse_raw:.3f} m")
#     print(f"RMSE avec RAIM                    : {rmse_raim:.3f} m")
#     print(f"RMSE sans RAIM pendant défaut     : {rmse_raw_fault:.3f} m")
#     print(f"RMSE avec RAIM pendant défaut     : {rmse_raim_fault:.3f} m")
#     print()
#     print(f"Détections RAIM totales           : {detections_total} / {len(t)}")
#     print(
#         "Détections pendant défaut         : "
#         f"{detections_fault_window} / {int(np.sum(fault_mask))}"
#     )

#     if valid_exclusions:
#         most_common = max(
#             set(valid_exclusions),
#             key=valid_exclusions.count,
#         )
#         print(f"Satellite le plus souvent exclu   : {most_common + 1}")

#     plt.figure(figsize=(12, 5))
#     plt.plot(t, error_raw, label="Sans RAIM", alpha=0.7)
#     plt.plot(t, error_raim, label="Avec RAIM/FDE", linewidth=2)
#     plt.axvspan(fault_start, fault_end, alpha=0.2, label="Fenêtre défaut")
#     plt.title("Impact RAIM/FDE sur l'erreur de position")
#     plt.xlabel("Temps (s)")
#     plt.ylabel("Erreur 3D (m)")
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.show()

#     plt.figure(figsize=(12, 4))
#     plt.plot(t, raim_statistics, label="Statistique RAIM")
#     plt.axhline(25.0, linestyle="--", label="Seuil RAIM")
#     plt.axvspan(fault_start, fault_end, alpha=0.2, label="Fenêtre défaut")
#     plt.title("Statistique RAIM")
#     plt.xlabel("Temps (s)")
#     plt.ylabel("Statistique")
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.show()

#     plt.figure(figsize=(12, 4))
#     plt.step(t, raim_fault_flags.astype(int), where="post")
#     plt.axvspan(fault_start, fault_end, alpha=0.2, label="Fenêtre défaut")
#     plt.title("Détection RAIM")
#     plt.xlabel("Temps (s)")
#     plt.ylabel("Défaut détecté")
#     plt.yticks([0, 1], ["Non", "Oui"])
#     plt.grid(True)
#     plt.legend()
#     plt.tight_layout()
#     plt.show()


# if __name__ == "__main__":
#     main()