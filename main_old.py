import matplotlib.pyplot as plt
import numpy as np

from src.simulation.trajectory import generate_drone_trajectory
from src.gnss.constellation_generator import generate_multi_constellation
from src.gnss.pseudorange import compute_pseudoranges
from src.gnss.dop import compute_dop
from src.gnss.raim import raim_fde

from src.signal_processing.noise import (
    gaussian_noise,
    constant_bias,
    satellite_bias,
    multipath_sinusoidal,
)

from src.gnss.gauss_newton import solve_position_gauss_newton
from src.fusion.kalman import KalmanFilter3D
from src.sensors.imu import simulate_accelerometer
from src.fusion.fusion_kalman import GNSSINSKalman
from src.sensors.gyroscope import simulate_gyroscope
from src.simulation.kinematics import compute_kinematic_state
from src.ins.mechanization import run_strapdown_ins


def format_3d_axis(ax):
    ax.set_xlim(-100, 700)
    ax.set_ylim(-200, 200)
    ax.set_zlim(80, 170)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.view_init(elev=25, azim=-60)


def compute_rmse(error):
    return np.sqrt(np.mean(error**2))


def main():

    dt = 0.1
    # ==========================================================
    # 1. Trajectoire réelle
    # ==========================================================

    t, position, velocity, acceleration = generate_drone_trajectory(
        duration=240.0,
        dt=dt
    )

    # ==========================================================
    # 2. Affichage trajectoire réelle
    # ==========================================================

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(position[:, 0], position[:, 1], position[:, 2],
            linewidth=2, label="Trajectoire")

    ax.scatter(position[0, 0], position[0, 1], position[0, 2],
               color="green", s=80, label="Départ")

    ax.scatter(position[-1, 0], position[-1, 1], position[-1, 2],
               color="red", s=80, label="Arrivée")

    ax.set_title("Trajectoire 3D simulée du drone")
    format_3d_axis(ax)
    ax.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 3. Vitesse / accélération
    # ==========================================================

    speed = np.linalg.norm(velocity, axis=1)

    plt.figure(figsize=(10, 4))
    plt.plot(t, speed)
    plt.title("Norme de la vitesse du drone")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (m/s)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    acc_norm = np.linalg.norm(acceleration, axis=1)

    plt.figure(figsize=(10, 4))
    plt.plot(t, acc_norm)
    plt.title("Norme de l'accélération du drone")
    plt.xlabel("Temps (s)")
    plt.ylabel("Accélération (m/s²)")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 4. Constellation GNSS paramétrable
    # ==========================================================
    # La constellation n'est plus codée en dur.
    # Elle est générée à partir d'un modèle multi-GNSS simplifié.
    #
    # Exemple utilisé ici :
    # - 8 satellites GPS ;
    # - 6 satellites Galileo ;
    # - 0 GLONASS ;
    # - 0 BeiDou.
    #
    # Cette configuration améliore généralement la géométrie
    # satellite par rapport à une constellation fixe de 6 satellites.

    satellites = generate_multi_constellation(
        gps=8,
        galileo=6,
        glonass=0,
        beidou=0,
        seed=42
    )

    print("=" * 60)
    print("Constellation GNSS générée")
    print("=" * 60)
    print("\nConfiguration : GPS + Galileo")
    print(f"Nombre de satellites : {len(satellites)}")
    print("\nCoordonnées des satellites (ECEF) :\n")
    print(np.round(satellites, 2))

    # ==========================================================
    # 5. Qualité géométrique GNSS - DOP
    # ==========================================================
    # Les indicateurs DOP permettent d'évaluer la géométrie de
    # la constellation GNSS par rapport au récepteur.
    #
    # GDOP : qualité globale position + temps
    # PDOP : qualité position 3D
    # HDOP : qualité horizontale
    # VDOP : qualité verticale
    # TDOP : qualité temporelle / horloge
    #
    # Plus la valeur est faible, meilleure est la géométrie.

    dop_first = compute_dop(
        receiver_position=position[0],
        satellites=satellites
    )

    print("\nIndicateurs DOP au premier instant :")
    for name, value in dop_first.items():
        print(f"{name} : {value:.3f}")

    dop_history = {
        "GDOP": [],
        "PDOP": [],
        "HDOP": [],
        "VDOP": [],
        "TDOP": [],
    }

    for receiver_k in position:
        dop_k = compute_dop(
            receiver_position=receiver_k,
            satellites=satellites
        )

        for name in dop_history:
            dop_history[name].append(dop_k[name])

    for name in dop_history:
        dop_history[name] = np.array(dop_history[name])

    plt.figure(figsize=(10, 5))

    for name, values in dop_history.items():
        plt.plot(t, values, label=name)

    plt.title("Évolution des indicateurs DOP GNSS")
    plt.xlabel("Temps (s)")
    plt.ylabel("DOP")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 6. Pseudodistances au premier instant
    # ==========================================================

    receiver = position[0]

    first_pseudoranges = compute_pseudoranges(
        receiver,
        satellites,
        clock_bias_seconds=0.0
    )

    print("\nPosition du récepteur (premier instant) :\n")
    print(receiver)

    print("\nPseudodistances au premier instant (m) :\n")
    print(np.round(first_pseudoranges, 2))

    # ==========================================================
    # 7. Pseudodistances sur toute la trajectoire
    # ==========================================================

    pseudoranges = []

    for receiver_k in position:
        rho_k = compute_pseudoranges(
            receiver_k,
            satellites,
            clock_bias_seconds=0.0
        )
        pseudoranges.append(rho_k)

    pseudoranges = np.array(pseudoranges)

    print("\nDimensions de la matrice des pseudodistances :")
    print(pseudoranges.shape)

    # ==========================================================
    # 8. Constellation au premier instant
    # ==========================================================

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(position[0, 0], position[0, 1], position[0, 2],
               color="red", s=100, label="Drone")

    ax.scatter(satellites[:, 0], satellites[:, 1], satellites[:, 2],
               color="blue", s=60, label="Satellites")

    for sat in satellites:
        ax.plot(
            [position[0, 0], sat[0]],
            [position[0, 1], sat[1]],
            [position[0, 2], sat[2]],
            "--",
            alpha=0.4
        )

    ax.set_title("Constellation GNSS générée et distances géométriques")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 9. Évolution des pseudodistances
    # ==========================================================

    plt.figure(figsize=(10, 5))

    for i in range(satellites.shape[0]):
        plt.plot(t, pseudoranges[:, i], label=f"Satellite {i + 1}")

    plt.title("Évolution des pseudodistances GNSS")
    plt.xlabel("Temps (s)")
    plt.ylabel("Pseudodistance (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 10. Variation des pseudodistances
    # ==========================================================

    delta_pseudoranges = pseudoranges - pseudoranges[0, :]

    plt.figure(figsize=(10, 5))

    for i in range(satellites.shape[0]):
        plt.plot(t, delta_pseudoranges[:, i], label=f"Satellite {i + 1}")

    plt.title("Variation des pseudodistances GNSS")
    plt.xlabel("Temps (s)")
    plt.ylabel("Variation de pseudodistance (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 11. Bruits GNSS
    # ==========================================================

    noise_gaussian = gaussian_noise(
        pseudoranges.shape,
        sigma=2.0,
        seed=42
    )

    noise_bias = constant_bias(
        pseudoranges.shape,
        bias=10.0
    )

    noise_satellite_bias = satellite_bias(
        pseudoranges.shape,
        satellite_index=2,
        bias=30.0
    )

    noise_multipath = multipath_sinusoidal(
        t,
        n_satellites=satellites.shape[0],
        omega=0.05
    )

    # Mesures GNSS nominales : bruit blanc + multipath.
    pseudoranges_noisy_nominal = (
        pseudoranges
        + noise_gaussian
        + noise_multipath
    )

    # Mesures GNSS avec défaut satellite volontaire.
    # Ce défaut permet de tester la capacité RAIM à détecter et
    # exclure une mesure incohérente avant l'estimation de position.
    pseudoranges_noisy = (
        pseudoranges_noisy_nominal
        + noise_satellite_bias
    )

    print("\nDimensions des pseudodistances bruitées :")
    print(pseudoranges_noisy.shape)

    print("\nDéfaut RAIM simulé :")
    print("Satellite fautif : 3")
    print("Biais ajouté     : 30.0 m")

    # ==========================================================
    # 12. Visualisation du bruit
    # ==========================================================

    error_added = pseudoranges_noisy - pseudoranges

    plt.figure(figsize=(10, 5))

    for i in range(satellites.shape[0]):
        plt.plot(t, error_added[:, i], label=f"Satellite {i + 1}")

    plt.title("Erreurs GNSS ajoutées aux pseudodistances avec défaut satellite")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 13. Gauss-Newton sans RAIM et solution protégée RAIM
    # ==========================================================
    # Deux estimations sont calculées :
    #
    # 1. Gauss-Newton non protégé :
    #    utilise toutes les pseudodistances, y compris le satellite
    #    fautif simulé.
    #
    # 2. RAIM/FDE :
    #    effectue un test d'intégrité, tente d'exclure le satellite
    #    fautif, puis fournit une solution GNSS protégée.

    estimated_positions_gn_raw = []
    estimated_positions_gn = []

    raim_statistics = []
    raim_thresholds = []
    raim_flags = []
    excluded_satellites = []

    initial_position_raw = position[0] + np.array([50.0, -50.0, 20.0])
    initial_position_raim = initial_position_raw.copy()

    for k in range(len(t)):

        estimated_position_raw_k, _ = solve_position_gauss_newton(
            satellites=satellites,
            pseudoranges=pseudoranges_noisy[k],
            initial_position=initial_position_raw,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_positions_gn_raw.append(estimated_position_raw_k)
        initial_position_raw = estimated_position_raw_k

        raim_result_k = raim_fde(
            satellites=satellites,
            pseudoranges=pseudoranges_noisy[k],
            initial_position=initial_position_raim,
            sigma=2.0,
            pfa=1e-3,
            max_iterations=30,
            tolerance=1e-4,
        )

        estimated_position_raim_k = raim_result_k["position"]

        estimated_positions_gn.append(estimated_position_raim_k)
        initial_position_raim = estimated_position_raim_k

        raim_statistics.append(raim_result_k["statistic"])
        raim_thresholds.append(raim_result_k["threshold"])
        raim_flags.append(raim_result_k["fault_detected"])
        excluded_satellites.append(raim_result_k["excluded_satellite"])

    estimated_positions_gn_raw = np.array(estimated_positions_gn_raw)
    estimated_positions_gn = np.array(estimated_positions_gn)

    raim_statistics = np.array(raim_statistics)
    raim_thresholds = np.array(raim_thresholds)
    raim_flags = np.array(raim_flags, dtype=bool)

    print("\nDimensions des positions estimées par Gauss-Newton brut :")
    print(estimated_positions_gn_raw.shape)

    print("\nDimensions des positions protégées RAIM :")
    print(estimated_positions_gn.shape)

    error_gn_raw = np.linalg.norm(
        estimated_positions_gn_raw - position,
        axis=1,
    )

    error_gn = np.linalg.norm(
        estimated_positions_gn - position,
        axis=1,
    )

    rmse_gn_raw = compute_rmse(error_gn_raw)
    rmse_gn = compute_rmse(error_gn)

    n_raim_faults = int(np.sum(raim_flags))

    valid_exclusions = [
        index for index in excluded_satellites
        if index is not None
    ]

    print(f"\nRMSE Gauss-Newton sans RAIM : {rmse_gn_raw:.3f} m")
    print(f"RMSE Gauss-Newton avec RAIM : {rmse_gn:.3f} m")
    print(f"Détections RAIM             : {n_raim_faults} / {len(t)}")

    if valid_exclusions:
        most_common_excluded = max(
            set(valid_exclusions),
            key=valid_exclusions.count,
        )
        print(f"Satellite le plus souvent exclu : {most_common_excluded + 1}")

    plt.figure(figsize=(10, 5))
    plt.plot(
        t,
        error_gn_raw,
        label="Gauss-Newton sans RAIM",
        alpha=0.6,
    )
    plt.plot(
        t,
        error_gn,
        label="Gauss-Newton avec RAIM/FDE",
        linewidth=2,
    )
    plt.title("Erreur de position - Impact RAIM/FDE")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(10, 4))
    plt.plot(t, raim_statistics, label="Statistique χ²")
    plt.plot(t, raim_thresholds, "--", linewidth=2, label="Seuil χ²")
    plt.title("Statistique RAIM globale")
    plt.xlabel("Temps (s)")
    plt.ylabel("Statistique RAIM")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 14. Vérification numérique
    # ==========================================================

    print("\n================ Vérification =================")
    print("\nTrajectoire vraie")
    print("Départ :", position[0])
    print("Arrivée :", position[-1])

    print("\nEstimation GNSS protégée RAIM")
    print("Départ :", estimated_positions_gn[0])
    print("Arrivée :", estimated_positions_gn[-1])

    print("\nAmplitude trajectoire vraie")
    print("xmin xmax :", position[:, 0].min(), position[:, 0].max())
    print("ymin ymax :", position[:, 1].min(), position[:, 1].max())
    print("zmin zmax :", position[:, 2].min(), position[:, 2].max())

    # ==========================================================
    # 15. Trajectoire vraie vs Gauss-Newton
    # ==========================================================

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(position[:, 0], position[:, 1], position[:, 2],
            label="Trajectoire vraie", linewidth=2)

    ax.plot(estimated_positions_gn_raw[:, 0],
            estimated_positions_gn_raw[:, 1],
            estimated_positions_gn_raw[:, 2],
            label="Gauss-Newton sans RAIM", alpha=0.35)

    ax.plot(estimated_positions_gn[:, 0],
            estimated_positions_gn[:, 1],
            estimated_positions_gn[:, 2],
            label="Gauss-Newton avec RAIM", linewidth=2)

    ax.scatter(position[0, 0], position[0, 1], position[0, 2],
               color="green", s=80, label="Départ")

    ax.scatter(position[-1, 0], position[-1, 1], position[-1, 2],
               color="red", s=80, label="Arrivée")

    ax.set_title("Trajectoire vraie vs estimation GNSS Gauss-Newton")
    format_3d_axis(ax)
    ax.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 16. Kalman GNSS
    # ==========================================================

    kf_no_outage = KalmanFilter3D(
        dt=dt,
        process_noise=0.1,
        measurement_noise=25.0
    )

    kf_no_outage.initialize(estimated_positions_gn[0])

    estimated_positions_kf_no_outage = []

    for measurement in estimated_positions_gn:
        kf_no_outage.predict()
        position_kf = kf_no_outage.update(measurement)
        estimated_positions_kf_no_outage.append(position_kf)

    estimated_positions_kf_no_outage = np.array(
        estimated_positions_kf_no_outage
    )

    error_kf_no_outage = np.linalg.norm(
        estimated_positions_kf_no_outage - position,
        axis=1
    )

    rmse_kf_no_outage = compute_rmse(error_kf_no_outage)

    print(f"RMSE Kalman GNSS : {rmse_kf_no_outage:.3f} m")

    # ==========================================================
    # 17. Comparaison Gauss-Newton / Kalman
    # ==========================================================

    plt.figure(figsize=(10, 5))
    plt.plot(t, error_gn, label="Gauss-Newton avec RAIM", alpha=0.5)
    plt.plot(t, error_kf_no_outage, label="Kalman GNSS", linewidth=2)
    plt.title("Erreur de position : Gauss-Newton vs Kalman")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 18. Trajectoire GNSS / Kalman
    # ==========================================================

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(position[:, 0], position[:, 1], position[:, 2],
            label="Trajectoire vraie", linewidth=2)

    ax.plot(estimated_positions_gn[:, 0],
            estimated_positions_gn[:, 1],
            estimated_positions_gn[:, 2],
            label="Gauss-Newton avec RAIM", alpha=0.25)

    ax.plot(estimated_positions_kf_no_outage[:, 0],
            estimated_positions_kf_no_outage[:, 1],
            estimated_positions_kf_no_outage[:, 2],
            label="Kalman GNSS", linewidth=2)

    ax.set_title("Trajectoire vraie vs GNSS brut vs Kalman")
    format_3d_axis(ax)
    ax.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # NOTE IMPORTANTE - Version stable INS/GNSS
    # ==========================================================
    # Dans cette version stable, l'INS strapdown est validée avec
    # les attitudes de référence issues du simulateur cinématique.
    #
    # Cela permet de valider proprement :
    # - la génération de force spécifique dans le repère body ;
    # - la compensation de la gravité ;
    # - la rotation body -> navigation ;
    # - l'intégration vitesse / position ;
    # - la fusion GNSS/INS en conditions nominales GNSS.
    #
    # Les biais IMU sont conservés comme scénario d'analyse.
    # Leur estimation avancée pourra être ajoutée plus tard
    # avec un filtre d'état d'erreur complet.

    # ==========================================================
    # 19. Cinématique complète pour simulation IMU
    # ==========================================================

    kinematic_state = compute_kinematic_state(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        dt=dt
    )

    specific_force_body = kinematic_state["specific_force_body"]
    true_angular_rate = kinematic_state["angular_rates"]
    initial_attitude = kinematic_state["quaternions"][0]

    # ==========================================================
    # 20. Scénarios IMU / INS / Fusion
    # ==========================================================
    # Deux configurations sont évaluées :
    #
    # Scénario A - IMU nominale :
    #   - faible bruit ;
    #   - aucun biais ;
    #   - sert à valider la chaîne strapdown et la fusion.
    #
    # Scénario B - IMU bruitée / biaisée :
    #   - bruit plus élevé ;
    #   - biais accéléromètre et gyroscope ;
    #   - illustre la dérive inertielle en l'absence d'estimation
    #     des biais IMU.

    imu_scenarios = {
        "nominal": {
            "label": "IMU nominale",
            "acc_sigma": 0.003,
            "acc_bias": np.array([0.0, 0.0, 0.0]),
            "gyro_sigma": 0.0001,
            "gyro_bias": np.array([0.0, 0.0, 0.0]),
        },
        "biased": {
            "label": "IMU bruitée / biaisée",
            "acc_sigma": 0.03,
            "acc_bias": np.array([0.01, -0.01, 0.005]),
            "gyro_sigma": 0.001,
            "gyro_bias": np.array([0.0, 0.0, 0.0005]),
        },
    }

    scenario_results = {}

    for scenario_name, scenario in imu_scenarios.items():

        print("\n" + "=" * 60)
        print(f"Scénario IMU : {scenario['label']}")
        print("=" * 60)

        # ------------------------------------------------------
        # Simulation accéléromètre
        # ------------------------------------------------------
        # L'accéléromètre reçoit une force spécifique exprimée
        # dans le repère body. Le bruit et les biais sont ajoutés
        # ici pour reproduire une mesure IMU.

        imu_acceleration = simulate_accelerometer(
            true_acceleration=specific_force_body,
            sigma=scenario["acc_sigma"],
            bias=scenario["acc_bias"],
            seed=42,
        )

        # ------------------------------------------------------
        # Simulation gyroscope
        # ------------------------------------------------------
        # Les vitesses angulaires sont exprimées en rad/s.
        # Le biais gyroscope est volontairement faible mais cumulatif.

        gyro_measurements = simulate_gyroscope(
            true_angular_rate=true_angular_rate,
            sigma=scenario["gyro_sigma"],
            bias=scenario["gyro_bias"],
            seed=42,
        )

        # ------------------------------------------------------
        # INS strapdown
        # ------------------------------------------------------
        # La version stable utilise les attitudes de référence du
        # simulateur cinématique pour isoler la validation de la
        # chaîne force spécifique -> navigation -> intégration.

        (
            strapdown_position,
            strapdown_velocity,
            strapdown_attitude,
        ) = run_strapdown_ins(
            initial_position=position[0],
            initial_velocity=velocity[0],
            initial_attitude=initial_attitude,
            accelerometer_measurements=imu_acceleration,
            gyroscope_measurements=gyro_measurements,
            dt=dt,
            reference_attitudes=kinematic_state["quaternions"],
        )

        error_strapdown = np.linalg.norm(
            strapdown_position - position,
            axis=1,
        )

        rmse_strapdown = compute_rmse(error_strapdown)

        # ------------------------------------------------------
        # Fusion GNSS / INS
        # ------------------------------------------------------

        fusion_no_outage = GNSSINSKalman()

        fusion_no_outage.initialize(
            position=position[0],
            velocity=velocity[0],
        )

        fusion_position_no_outage = np.zeros_like(position)
        fusion_velocity_no_outage = np.zeros_like(velocity)

        for k in range(len(t)):
            fusion_no_outage.predict(
                strapdown_position[k],
                strapdown_velocity[k],
            )

            fusion_no_outage.update(estimated_positions_gn[k])

            state = fusion_no_outage.state()

            fusion_position_no_outage[k] = state[:3]
            fusion_velocity_no_outage[k] = state[3:]

        error_fusion_no_outage = np.linalg.norm(
            fusion_position_no_outage - position,
            axis=1,
        )

        rmse_fusion_no_outage = compute_rmse(error_fusion_no_outage)

        scenario_results[scenario_name] = {
            "label": scenario["label"],
            "strapdown_position": strapdown_position,
            "strapdown_velocity": strapdown_velocity,
            "error_strapdown": error_strapdown,
            "rmse_strapdown": rmse_strapdown,
            "fusion_position_no_outage": fusion_position_no_outage,
            "fusion_velocity_no_outage": fusion_velocity_no_outage,
            "error_fusion_no_outage": error_fusion_no_outage,
            "rmse_fusion_no_outage": rmse_fusion_no_outage,
        }

        print(f"RMSE INS strapdown : {rmse_strapdown:.3f} m")
        print(
            "RMSE Fusion GNSS/INS : "
            f"{rmse_fusion_no_outage:.3f} m"
        )

    # ==========================================================
    # 21. Résumé final des performances
    # ==========================================================

    print("\n================ Résumé GNSS ================")
    print(f"Nombre de satellites              : {len(satellites)}")
    print(f"Gauss-Newton sans RAIM            : {rmse_gn_raw:.3f} m")
    print(f"Gauss-Newton avec RAIM            : {rmse_gn:.3f} m")
    print(f"Détections RAIM                   : {n_raim_faults} / {len(t)}")
    print(f"Kalman GNSS                      : {rmse_kf_no_outage:.3f} m")
    print(f"PDOP moyen                       : {np.mean(dop_history['PDOP']):.3f}")
    print(f"HDOP moyen                       : {np.mean(dop_history['HDOP']):.3f}")
    print(f"VDOP moyen                       : {np.mean(dop_history['VDOP']):.3f}")

    print("\n================ Résumé INS / Fusion ================")

    for scenario_name, result in scenario_results.items():
        print(f"\nScénario : {result['label']}")
        print(f"INS strapdown                     : {result['rmse_strapdown']:.3f} m")
        print(
            "Fusion GNSS/INS                  : "
            f"{result['rmse_fusion_no_outage']:.3f} m"
        )

    # ==========================================================
    # 22. Comparaison finale des erreurs
    # ==========================================================

    plt.figure(figsize=(12, 5))

    plt.plot(t, error_gn, label="Gauss-Newton avec RAIM", alpha=0.35)
    plt.plot(
        t,
        error_kf_no_outage,
        label="Kalman GNSS",
        linewidth=2,
    )
    for result in scenario_results.values():
        plt.plot(
            t,
            result["error_fusion_no_outage"],
            label=f"Fusion GNSS/INS - {result['label']}",
            linewidth=2,
        )

    plt.title("Comparaison finale des performances")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 23. Zoom : GNSS / Kalman / Fusion
    # ==========================================================

    plt.figure(figsize=(12, 5))

    plt.plot(t, error_gn, label="Gauss-Newton avec RAIM", alpha=0.25)
    plt.plot(
        t,
        error_kf_no_outage,
        label="Kalman GNSS",
        linewidth=2,
    )
    for result in scenario_results.values():
        plt.plot(
            t,
            result["error_fusion_no_outage"],
            label=f"Fusion GNSS/INS - {result['label']}",
            linewidth=2,
        )

    plt.ylim(0, 100)
    plt.title("Zoom : GNSS / Kalman / Fusion")
    plt.xlabel("Temps (s)")
    plt.ylabel("Erreur 3D (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ==========================================================
    # 24. Trajectoires finales
    # ==========================================================

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        position[:, 0],
        position[:, 1],
        position[:, 2],
        label="Trajectoire vraie",
        linewidth=2,
    )

    ax.plot(
        estimated_positions_gn_raw[:, 0],
        estimated_positions_gn_raw[:, 1],
        estimated_positions_gn_raw[:, 2],
        label="Gauss-Newton sans RAIM",
        alpha=0.12,
    )

    ax.plot(
        estimated_positions_gn[:, 0],
        estimated_positions_gn[:, 1],
        estimated_positions_gn[:, 2],
        label="Gauss-Newton avec RAIM",
        alpha=0.35,
    )

    ax.plot(
        estimated_positions_kf_no_outage[:, 0],
        estimated_positions_kf_no_outage[:, 1],
        estimated_positions_kf_no_outage[:, 2],
        label="Kalman GNSS",
        linewidth=2,
    )

    ax.plot(
        scenario_results["nominal"]["fusion_position_no_outage"][:, 0],
        scenario_results["nominal"]["fusion_position_no_outage"][:, 1],
        scenario_results["nominal"]["fusion_position_no_outage"][:, 2],
        label="Fusion GNSS/INS - IMU nominale",
        linewidth=3,
    )

    ax.plot(
        scenario_results["biased"]["fusion_position_no_outage"][:, 0],
        scenario_results["biased"]["fusion_position_no_outage"][:, 1],
        scenario_results["biased"]["fusion_position_no_outage"][:, 2],
        label="Fusion GNSS/INS - IMU bruitée / biaisée",
        linewidth=3,
    )

    ax.set_title("Trajectoires : vraie / GNSS / Fusion")
    format_3d_axis(ax)
    ax.legend()
    plt.tight_layout()
    plt.show()



if __name__ == "__main__":
    main()
