import numpy as np
import matplotlib.pyplot as plt

from src.simulation.trajectory import generate_drone_trajectory
from src.gnss.pseudorange import compute_pseudoranges
from src.gnss.gauss_newton import solve_position_gauss_newton
from src.gnss.dop import compute_dop
from src.gnss.constellation_generator import generate_multi_constellation
from src.signal_processing.noise import gaussian_noise, multipath_sinusoidal
from src.fusion.kalman import KalmanFilter3D


def compute_rmse(error):
    return np.sqrt(np.mean(error**2))


def evaluate_configuration(name, gps, galileo, glonass, beidou, position, t, dt):
    satellites = generate_multi_constellation(
        gps=gps,
        galileo=galileo,
        glonass=glonass,
        beidou=beidou,
        seed=42,
    )

    pseudoranges = np.array([
        compute_pseudoranges(p, satellites, clock_bias_seconds=0.0)
        for p in position
    ])

    noise_gaussian = gaussian_noise(
        pseudoranges.shape,
        sigma=2.0,
        seed=42,
    )

    noise_multipath = multipath_sinusoidal(
        t,
        n_satellites=satellites.shape[0],
        omega=0.05,
    )

    pseudoranges_noisy = (
        pseudoranges
        + noise_gaussian
        + noise_multipath
    )

    estimated_positions_gn = []
    initial_position = position[0] + np.array([50.0, -50.0, 20.0])

    for k in range(len(t)):
        estimated_position_k, _ = solve_position_gauss_newton(
            satellites=satellites,
            pseudoranges=pseudoranges_noisy[k],
            initial_position=initial_position,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_positions_gn.append(estimated_position_k)
        initial_position = estimated_position_k

    estimated_positions_gn = np.array(estimated_positions_gn)

    error_gn = np.linalg.norm(
        estimated_positions_gn - position,
        axis=1,
    )

    rmse_gn = compute_rmse(error_gn)

    kf = KalmanFilter3D(
        dt=dt,
        process_noise=0.1,
        measurement_noise=25.0,
    )

    kf.initialize(estimated_positions_gn[0])

    estimated_positions_kf = []

    for measurement in estimated_positions_gn:
        kf.predict()
        position_kf = kf.update(measurement)
        estimated_positions_kf.append(position_kf)

    estimated_positions_kf = np.array(estimated_positions_kf)

    error_kf = np.linalg.norm(
        estimated_positions_kf - position,
        axis=1,
    )

    rmse_kf = compute_rmse(error_kf)

    dop_values = []

    for receiver_position in position:
        dop = compute_dop(
            receiver_position=receiver_position,
            satellites=satellites,
        )
        dop_values.append(dop["PDOP"])

    pdop_mean = np.mean(dop_values)

    return {
        "name": name,
        "n_satellites": satellites.shape[0],
        "pdop": pdop_mean,
        "rmse_gauss_newton": rmse_gn,
        "rmse_kalman": rmse_kf,
    }


def main():
    dt = 0.1

    t, position, velocity, acceleration = generate_drone_trajectory(
        duration=240.0,
        dt=dt,
    )

    configurations = [
        ("GPS 6", 6, 0, 0, 0),
        ("GPS 8", 8, 0, 0, 0),
        ("GPS 12", 12, 0, 0, 0),
        ("GPS + Galileo", 8, 6, 0, 0),
        ("GPS + Galileo + BeiDou", 8, 6, 0, 6),
    ]

    results = []

    print("\n================ Benchmark constellations GNSS ================\n")

    for config in configurations:
        result = evaluate_configuration(
            name=config[0],
            gps=config[1],
            galileo=config[2],
            glonass=config[3],
            beidou=config[4],
            position=position,
            t=t,
            dt=dt,
        )

        results.append(result)

        print(
            f"{result['name']:<25} | "
            f"Sat: {result['n_satellites']:>2} | "
            f"PDOP: {result['pdop']:.3f} | "
            f"RMSE GN: {result['rmse_gauss_newton']:.3f} m | "
            f"RMSE KF: {result['rmse_kalman']:.3f} m"
        )

    names = [r["name"] for r in results]
    n_satellites = [r["n_satellites"] for r in results]
    pdop = [r["pdop"] for r in results]
    rmse_gn = [r["rmse_gauss_newton"] for r in results]
    rmse_kf = [r["rmse_kalman"] for r in results]

    plt.figure(figsize=(10, 5))
    plt.plot(n_satellites, pdop, marker="o")
    plt.title("PDOP moyen en fonction du nombre de satellites")
    plt.xlabel("Nombre de satellites")
    plt.ylabel("PDOP moyen")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(n_satellites, rmse_gn, marker="o", label="Gauss-Newton")
    plt.plot(n_satellites, rmse_kf, marker="o", label="Kalman GNSS")
    plt.title("RMSE GNSS en fonction de la constellation")
    plt.xlabel("Nombre de satellites")
    plt.ylabel("RMSE position 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(12, 5))
    x = np.arange(len(names))
    width = 0.35

    plt.bar(x - width / 2, rmse_gn, width, label="Gauss-Newton")
    plt.bar(x + width / 2, rmse_kf, width, label="Kalman GNSS")

    plt.xticks(x, names, rotation=20)
    plt.title("Comparaison RMSE par configuration GNSS")
    plt.ylabel("RMSE position 3D (m)")
    plt.grid(True, axis="y")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()