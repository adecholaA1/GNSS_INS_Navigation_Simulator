import numpy as np
import matplotlib.pyplot as plt

from src.simulation.trajectory import generate_drone_trajectory
from src.gnss.constellation_generator import generate_multi_constellation
from src.gnss.pseudorange import compute_pseudoranges
from src.gnss.raim import raim_fde
from src.gnss.fault_injection import inject_time_window_bias
from src.signal_processing.noise import (
    gaussian_noise,
    multipath_sinusoidal,
)


def rmse(x):
    return np.sqrt(np.mean(x ** 2))


def main():

    dt = 0.1

    # Probabilités de fausse alarme testées
    pfa_targets = [
        1e-1,
        5e-2,
        1e-2,
        5e-3,
        1e-3,
        5e-4,
        1e-4,
        1e-5,
    ]

    fault_satellite = 2
    fault_start = 60.0
    fault_end = 120.0
    fault_bias = 30.0

    t, true_position, _, _ = generate_drone_trajectory(
        duration=240,
        dt=dt,
    )

    satellites = generate_multi_constellation(
        gps=8,
        galileo=6,
        seed=42,
    )

    pseudoranges = np.array([
        compute_pseudoranges(
            p,
            satellites,
            clock_bias_seconds=0.0,
        )
        for p in true_position
    ])

    pseudoranges += gaussian_noise(
        pseudoranges.shape,
        sigma=2.0,
        seed=42,
    )

    pseudoranges += multipath_sinusoidal(
        t,
        satellites.shape[0],
        omega=0.05,
    )

    pseudoranges = inject_time_window_bias(
        pseudoranges,
        satellite_index=fault_satellite,
        start_time=fault_start,
        end_time=fault_end,
        bias=fault_bias,
        dt=dt,
    )

    fault_mask = (t >= fault_start) & (t <= fault_end)
    normal_mask = ~fault_mask

    results = []

    print()
    print("=" * 90)
    print("RAIM Statistical Benchmark (Chi²)")
    print("=" * 90)
    print()

    print(
        f"{'PFA cible':>12}"
        f"{'Chi²':>10}"
        f"{'PD':>10}"
        f"{'PFA':>10}"
        f"{'PMD':>10}"
        f"{'RMSE':>12}"
    )

    for pfa_target in pfa_targets:

        estimated = []
        detections = []

        x0 = true_position[0] + np.array([50.0, -50.0, 20.0])

        chi2_threshold = None

        for k in range(len(t)):

            result = raim_fde(
                satellites=satellites,
                pseudoranges=pseudoranges[k],
                initial_position=x0,
                sigma=2.0,
                pfa=pfa_target,
            )

            estimated.append(result["position"])
            detections.append(result["fault_detected"])

            chi2_threshold = result["threshold"]

            x0 = result["position"]

        estimated = np.array(estimated)
        detections = np.array(detections, dtype=bool)

        error = np.linalg.norm(
            estimated - true_position,
            axis=1,
        )

        rmse_value = rmse(error)

        pd = int(np.sum(detections[fault_mask]))
        pfa = int(np.sum(detections[normal_mask]))
        pmd = int(np.sum(~detections[fault_mask]))

        results.append(
            (
                pfa_target,
                chi2_threshold,
                pd,
                pfa,
                pmd,
                rmse_value,
            )
        )

        print(
            f"{pfa_target:12.1e}"
            f"{chi2_threshold:10.2f}"
            f"{pd:10d}"
            f"{pfa:10d}"
            f"{pmd:10d}"
            f"{rmse_value:12.3f}"
        )

    results = np.array(results)

    pfa_targets = results[:, 0]
    chi2_thresholds = results[:, 1]
    pd = results[:, 2]
    pfa = results[:, 3]
    pmd = results[:, 4]
    rmse_values = results[:, 5]

    plt.figure(figsize=(10, 5))
    plt.semilogx(
        pfa_targets,
        rmse_values,
        marker="o",
    )
    plt.grid(True)
    plt.xlabel("Probabilité de fausse alarme cible")
    plt.ylabel("RMSE (m)")
    plt.title("RMSE en fonction de la PFA cible")

    plt.figure(figsize=(10, 5))
    plt.semilogx(
        pfa_targets,
        pfa,
        marker="o",
    )
    plt.grid(True)
    plt.xlabel("Probabilité de fausse alarme cible")
    plt.ylabel("Nombre de fausses alarmes")
    plt.title("Fausses alarmes")

    plt.figure(figsize=(10, 5))
    plt.semilogx(
        pfa_targets,
        pmd,
        marker="o",
    )
    plt.grid(True)
    plt.xlabel("Probabilité de fausse alarme cible")
    plt.ylabel("Détections manquées")
    plt.title("Missed Detections")

    plt.figure(figsize=(10, 5))
    plt.semilogx(
        pfa_targets,
        pd,
        marker="o",
    )
    plt.grid(True)
    plt.xlabel("Probabilité de fausse alarme cible")
    plt.ylabel("Détections")
    plt.title("Probability of Detection")

    plt.figure(figsize=(10, 5))
    plt.semilogx(
        pfa_targets,
        chi2_thresholds,
        marker="o",
    )
    plt.grid(True)
    plt.xlabel("Probabilité de fausse alarme cible")
    plt.ylabel("Seuil χ²")
    plt.title("Seuil χ² utilisé par le RAIM")

    plt.show()


if __name__ == "__main__":
    main()




















# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.stats import chi2

# from src.simulation.trajectory import generate_drone_trajectory
# from src.gnss.constellation_generator import generate_multi_constellation
# from src.gnss.pseudorange import compute_pseudoranges
# from src.gnss.raim import raim_fde
# from src.gnss.fault_injection import inject_time_window_bias
# from src.signal_processing.noise import (
#     gaussian_noise,
#     multipath_sinusoidal,
# )


# def rmse(x):
#     return np.sqrt(np.mean(x ** 2))


# def main():

#     dt = 0.1

#     # Probabilités de fausse alarme testées
#     pfa_targets = [
#         1e-1,
#         5e-2,
#         1e-2,
#         5e-3,
#         1e-3,
#         5e-4,
#         1e-4,
#         1e-5,
#     ]

#     fault_satellite = 2
#     fault_start = 60.0
#     fault_end = 120.0
#     fault_bias = 30.0

#     t, true_position, _, _ = generate_drone_trajectory(
#         duration=240,
#         dt=dt,
#     )

#     satellites = generate_multi_constellation(
#         gps=8,
#         galileo=6,
#         seed=42,
#     )

#     pseudoranges = np.array([
#         compute_pseudoranges(
#             p,
#             satellites,
#             clock_bias_seconds=0.0,
#         )
#         for p in true_position
#     ])

#     pseudoranges += gaussian_noise(
#         pseudoranges.shape,
#         sigma=2.0,
#         seed=42,
#     )

#     pseudoranges += multipath_sinusoidal(
#         t,
#         satellites.shape[0],
#         omega=0.05,
#     )

#     pseudoranges = inject_time_window_bias(
#         pseudoranges,
#         satellite_index=fault_satellite,
#         start_time=fault_start,
#         end_time=fault_end,
#         bias=fault_bias,
#         dt=dt,
#     )

#     fault_mask = (t >= fault_start) & (t <= fault_end)
#     normal_mask = ~fault_mask

#     dof = satellites.shape[0] - 4

#     results = []

#     print()
#     print("=" * 90)
#     print("RAIM Statistical Benchmark (Chi²)")
#     print("=" * 90)
#     print()

#     print(
#         f"{'PFA cible':>12}"
#         f"{'Chi²':>10}"
#         f"{'PD':>10}"
#         f"{'PFA':>10}"
#         f"{'PMD':>10}"
#         f"{'RMSE':>12}"
#     )

#     for pfa_target in pfa_targets:

#         chi2_threshold = chi2.ppf(
#             1.0 - pfa_target,
#             dof,
#         )

#         estimated = []
#         detections = []

#         x0 = true_position[0] + np.array([50, -50, 20])

#         for k in range(len(t)):

#             result = raim_fde(
#                 satellites=satellites,
#                 pseudoranges=pseudoranges[k],
#                 initial_position=x0,
#                 sigma=2.0,
#                 threshold=chi2_threshold,
#             )

#             estimated.append(result["position"])
#             detections.append(result["fault_detected"])

#             x0 = result["position"]

#         estimated = np.array(estimated)
#         detections = np.array(detections)

#         error = np.linalg.norm(
#             estimated - true_position,
#             axis=1,
#         )

#         rmse_value = rmse(error)

#         pd = np.sum(detections[fault_mask])
#         pfa = np.sum(detections[normal_mask])
#         pmd = np.sum(~detections[fault_mask])

#         results.append(
#             (
#                 pfa_target,
#                 chi2_threshold,
#                 pd,
#                 pfa,
#                 pmd,
#                 rmse_value,
#             )
#         )

#         print(
#             f"{pfa_target:12.1e}"
#             f"{chi2_threshold:10.2f}"
#             f"{pd:10d}"
#             f"{pfa:10d}"
#             f"{pmd:10d}"
#             f"{rmse_value:12.3f}"
#         )

#     results = np.array(results)

#     pfa_targets = results[:, 0]
#     chi2_thresholds = results[:, 1]
#     pd = results[:, 2]
#     pfa = results[:, 3]
#     pmd = results[:, 4]
#     rmse_values = results[:, 5]

#     plt.figure(figsize=(10, 5))
#     plt.semilogx(
#         pfa_targets,
#         rmse_values,
#         marker="o",
#     )
#     plt.grid(True)
#     plt.xlabel("Probabilité de fausse alarme cible")
#     plt.ylabel("RMSE (m)")
#     plt.title("RMSE en fonction de la PFA cible")

#     plt.figure(figsize=(10, 5))
#     plt.semilogx(
#         pfa_targets,
#         pfa,
#         marker="o",
#     )
#     plt.grid(True)
#     plt.xlabel("Probabilité de fausse alarme cible")
#     plt.ylabel("Fausses alarmes")
#     plt.title("Fausses alarmes")

#     plt.figure(figsize=(10, 5))
#     plt.semilogx(
#         pfa_targets,
#         pmd,
#         marker="o",
#     )
#     plt.grid(True)
#     plt.xlabel("Probabilité de fausse alarme cible")
#     plt.ylabel("Détections manquées")
#     plt.title("Missed Detections")

#     plt.figure(figsize=(10, 5))
#     plt.semilogx(
#         pfa_targets,
#         pd,
#         marker="o",
#     )
#     plt.grid(True)
#     plt.xlabel("Probabilité de fausse alarme cible")
#     plt.ylabel("Détections")
#     plt.title("Probability of Detection")

#     plt.show()


# if __name__ == "__main__":
#     main()












# # import numpy as np
# # import matplotlib.pyplot as plt

# # from src.simulation.trajectory import generate_drone_trajectory
# # from src.gnss.constellation_generator import generate_multi_constellation
# # from src.gnss.pseudorange import compute_pseudoranges
# # from src.gnss.gauss_newton import solve_position_gauss_newton
# # from src.gnss.raim import raim_fde
# # from src.gnss.fault_injection import inject_time_window_bias
# # from src.signal_processing.noise import gaussian_noise, multipath_sinusoidal


# # def rmse(x):
# #     return np.sqrt(np.mean(x ** 2))


# # def main():

# #     dt = 0.1

# #     thresholds = np.arange(5, 55, 5)

# #     fault_satellite = 2
# #     fault_start = 60.0
# #     fault_end = 120.0
# #     fault_bias = 30.0

# #     t, true_position, _, _ = generate_drone_trajectory(
# #         duration=240,
# #         dt=dt,
# #     )

# #     satellites = generate_multi_constellation(
# #         gps=8,
# #         galileo=6,
# #         seed=42,
# #     )

# #     pseudoranges = np.array([
# #         compute_pseudoranges(
# #             p,
# #             satellites,
# #             clock_bias_seconds=0.0,
# #         )
# #         for p in true_position
# #     ])

# #     pseudoranges += gaussian_noise(
# #         pseudoranges.shape,
# #         sigma=2.0,
# #         seed=42,
# #     )

# #     pseudoranges += multipath_sinusoidal(
# #         t,
# #         satellites.shape[0],
# #         omega=0.05,
# #     )

# #     pseudoranges = inject_time_window_bias(
# #         pseudoranges,
# #         satellite_index=fault_satellite,
# #         start_time=fault_start,
# #         end_time=fault_end,
# #         bias=fault_bias,
# #         dt=dt,
# #     )

# #     fault_mask = (t >= fault_start) & (t <= fault_end)
# #     normal_mask = ~fault_mask

# #     results = []

# #     print()
# #     print("=" * 80)
# #     print("RAIM Threshold Benchmark")
# #     print("=" * 80)
# #     print()

# #     print(
# #         f"{'Threshold':>10}"
# #         f"{'PD':>10}"
# #         f"{'PFA':>10}"
# #         f"{'PMD':>10}"
# #         f"{'RMSE':>12}"
# #     )

# #     for threshold in thresholds:

# #         estimated = []

# #         detections = []

# #         x0 = true_position[0] + np.array([50, -50, 20])

# #         for k in range(len(t)):

# #             result = raim_fde(
# #                 satellites=satellites,
# #                 pseudoranges=pseudoranges[k],
# #                 initial_position=x0,
# #                 sigma=2.0,
# #                 threshold=threshold,
# #             )

# #             estimated.append(result["position"])
# #             detections.append(result["fault_detected"])

# #             x0 = result["position"]

# #         estimated = np.array(estimated)
# #         detections = np.array(detections)

# #         error = np.linalg.norm(
# #             estimated - true_position,
# #             axis=1,
# #         )

# #         rmse_value = rmse(error)

# #         pd = np.sum(detections[fault_mask])

# #         pfa = np.sum(detections[normal_mask])

# #         pmd = np.sum(~detections[fault_mask])

# #         results.append(
# #             (
# #                 threshold,
# #                 pd,
# #                 pfa,
# #                 pmd,
# #                 rmse_value,
# #             )
# #         )

# #         print(
# #             f"{threshold:10d}"
# #             f"{pd:10d}"
# #             f"{pfa:10d}"
# #             f"{pmd:10d}"
# #             f"{rmse_value:12.3f}"
# #         )

# #     results = np.array(results)

# #     threshold = results[:, 0]
# #     pd = results[:, 1]
# #     pfa = results[:, 2]
# #     pmd = results[:, 3]
# #     rmse_values = results[:, 4]

# #     plt.figure(figsize=(10, 5))
# #     plt.plot(threshold, rmse_values, marker="o")
# #     plt.grid(True)
# #     plt.xlabel("RAIM Threshold")
# #     plt.ylabel("RMSE (m)")
# #     plt.title("RMSE en fonction du seuil RAIM")

# #     plt.figure(figsize=(10, 5))
# #     plt.plot(threshold, pfa, marker="o")
# #     plt.grid(True)
# #     plt.xlabel("RAIM Threshold")
# #     plt.ylabel("False Alarms")
# #     plt.title("Fausses alarmes")

# #     plt.figure(figsize=(10, 5))
# #     plt.plot(threshold, pmd, marker="o")
# #     plt.grid(True)
# #     plt.xlabel("RAIM Threshold")
# #     plt.ylabel("Missed Detections")
# #     plt.title("Détections manquées")

# #     plt.figure(figsize=(10, 5))
# #     plt.plot(threshold, pd, marker="o")
# #     plt.grid(True)
# #     plt.xlabel("RAIM Threshold")
# #     plt.ylabel("Detected Faults")
# #     plt.title("Détections RAIM")

# #     plt.show()


# # if __name__ == "__main__":
# #     main()