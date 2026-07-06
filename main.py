"""
main.py

Démonstration principale du simulateur GNSS/INS.

Ce fichier orchestre :
- trajectoire vraie ;
- simulation GNSS ;
- RAIM/FDE ;
- Kalman GNSS ;
- INS Strapdown ;
- fusion GNSS/INS ;
- animation complète ;
- synthèse finale.
"""




from src.pipeline.trajectory_pipeline import run_trajectory_pipeline
from src.pipeline.gnss_pipeline import run_gnss_pipeline
from src.pipeline.raim_pipeline import run_raim_pipeline
from src.pipeline.kalman_pipeline import run_kalman_pipeline
from src.pipeline.ins_pipeline import run_ins_pipeline
from src.pipeline.fusion_pipeline import run_fusion_pipeline

from src.visualization.style import apply_style
from src.visualization.terminal import (
    print_banner,
    print_step,
    print_final_summary,
)
from src.visualization.animation import animate_navigation_pipeline
from src.visualization.plots import (
    plot_final_navigation_summary,
    show_all,
)
from src.visualization.dashboard import show_dashboard

from src.utils.export import create_results_directories

create_results_directories()


def main():

    apply_style()
    print_banner()

    trajectory = run_trajectory_pipeline(
        duration=240.0,
        dt=0.1,
    )
    print_step("Trajectoire vraie générée")

    gnss = run_gnss_pipeline(
        trajectory=trajectory,
        gps=8,
        galileo=6,
        glonass=0,
        beidou=0,
        seed=42,
        pseudorange_sigma=2.0,
        multipath_omega=0.05,
    )
    print_step("Constellation GNSS et pseudodistances générées")

    raim = run_raim_pipeline(
        trajectory=trajectory,
        gnss=gnss,
        fault_satellite=2,
        fault_bias=30.0,
        sigma=2.0,
        pfa=1e-3,
    )
    print_step("RAIM/FDE exécuté")

    kalman = run_kalman_pipeline(
        trajectory=trajectory,
        raim=raim,
        process_noise=0.1,
        measurement_noise=25.0,
    )
    print_step("Filtre de Kalman GNSS exécuté")

    ins = run_ins_pipeline(
        trajectory=trajectory,
        seed=42,
    )
    print_step("Navigation INS Strapdown exécutée")

    fusion = run_fusion_pipeline(
        trajectory=trajectory,
        kalman=kalman,
        ins=ins,
    )
    print_step("Fusion GNSS/INS exécutée")

    print_final_summary(
        gnss=gnss,
        raim=raim,
        kalman=kalman,
        ins=ins,
        fusion=fusion,
    )

    true_position = trajectory["position"]
    t = trajectory["t"]

    # animate_navigation_pipeline(
    #     true_position=true_position,
    #     satellites=gnss["satellites"],
    #     gnss_position=raim["estimated_raw"],
    #     raim_position=raim["estimated_raim"],
    #     kalman_position=kalman["estimated_positions"],
    #     ins_position=ins["noisy"]["position"],
    #     fusion_position=fusion["noisy"]["position"],
    #     raim_flags=raim["flags"],
    #     excluded_satellites=raim["excluded_satellites"],
    #     interval=10,
    #     step=5,
    # )

    animate_navigation_pipeline(
        true_position=true_position,
        satellites=gnss["satellites"],
        gnss_position=raim["estimated_raw"],
        raim_position=raim["estimated_raim"],
        kalman_position=kalman["estimated_positions"],
        ins_position=ins["noisy"]["position"],
        fusion_position=fusion["nominal"]["position"],
        raim_flags=raim["flags"],
        excluded_satellites=raim["excluded_satellites"],
        interval=10,
        step=5,
    )

    # plot_final_navigation_summary(
    #     t=t,
    #     true_position=true_position,
    #     gnss_position=raim["estimated_raw"],
    #     raim_position=raim["estimated_raim"],
    #     kalman_position=kalman["estimated_positions"],
    #     ins_position=ins["noisy"]["position"],
    #     fusion_position=fusion["noisy"]["position"],
    #     gnss_error=raim["error_raw"],
    #     raim_error=raim["error_raim"],
    #     kalman_error=kalman["error"],
    #     ins_error=ins["noisy"]["error"],
    #     fusion_error=fusion["noisy"]["error"],
    # )


    # plot_final_navigation_summary(
    #     t=t,
    #     true_position=true_position,
    #     gnss_position=raim["estimated_raw"],
    #     raim_position=raim["estimated_raim"],
    #     kalman_position=kalman["estimated_positions"],
    #     gnss_error=raim["error_raw"],
    #     raim_error=raim["error_raim"],
    #     kalman_error=kalman["error"],
    #     ins_position=ins["nominal"]["position"],
    #     fusion_position=fusion["nominal"]["position"],
    #     ins_error=ins["nominal"]["error"],
    #     fusion_error=fusion["nominal"]["error"]
    # )

    plot_final_navigation_summary(
        t=t,
        true_position=true_position,
        gnss_position=raim["estimated_raw"],
        raim_position=raim["estimated_raim"],
        kalman_position=kalman["estimated_positions"],
        ins_nominal_position=ins["nominal"]["position"],
        fusion_nominal_position=fusion["nominal"]["position"],
        ins_noisy_position=ins["noisy"]["position"],
        fusion_noisy_position=fusion["noisy"]["position"],
        gnss_error=raim["error_raw"],
        raim_error=raim["error_raim"],
        kalman_error=kalman["error"],
        ins_nominal_error=ins["nominal"]["error"],
        fusion_nominal_error=fusion["nominal"]["error"],
        ins_noisy_error=ins["noisy"]["error"],
        fusion_noisy_error=fusion["noisy"]["error"],
    )

    show_all()

    excluded_satellite = None

    if raim["most_common_excluded"] is not None:
        excluded_satellite = raim["most_common_excluded"] + 1

    show_dashboard(
        configuration=gnss["configuration"],
        n_satellites=gnss["satellites"].shape[0],
        pdop=gnss["dop_mean"]["PDOP"],
        hdop=gnss["dop_mean"]["HDOP"],
        vdop=gnss["dop_mean"]["VDOP"],
        gn_rmse=raim["rmse_raw"],
        raim_rmse=raim["rmse_raim"],
        kalman_rmse=kalman["rmse"],
        ins_nominal_rmse=ins["nominal"]["rmse"],
        fusion_nominal_rmse=fusion["nominal"]["rmse"],
        ins_noisy_rmse=ins["noisy"]["rmse"],
        fusion_noisy_rmse=fusion["noisy"]["rmse"],
        raim_detected=raim["n_detections"] > 0,
        excluded_satellite=excluded_satellite,
    )

    # from src.visualization.viewer import NavigationViewer

    # viewer = NavigationViewer()

    # viewer.load(
    #     trajectory=trajectory,
    #     gnss=gnss,
    #     raim=raim,
    #     kalman=kalman,
    #     ins=ins,
    #     fusion=fusion,
    # )

    # viewer.run()


if __name__ == "__main__":
    main()




















# """
# main.py

# Point d'entrée principal du simulateur GNSS/INS.

# Ce fichier orchestre :
# - la trajectoire vraie ;
# - la simulation GNSS ;
# - RAIM/FDE ;
# - Kalman GNSS ;
# - INS Strapdown ;
# - Fusion GNSS/INS ;
# - visualisations ;
# - dashboard final.
# """

# import numpy as np

# from src.pipeline.trajectory_pipeline import run_trajectory_pipeline
# from src.pipeline.gnss_pipeline import run_gnss_pipeline
# from src.pipeline.raim_pipeline import run_raim_pipeline
# from src.pipeline.kalman_pipeline import run_kalman_pipeline
# from src.pipeline.ins_pipeline import run_ins_pipeline
# from src.pipeline.fusion_pipeline import run_fusion_pipeline

# from src.visualization.animation import (
#     animate_true_trajectory,
#     animate_constellation,
# )

# from src.visualization.plots import (
#     plot_trajectory,
#     plot_position_error,
#     plot_raim_statistics,
#     plot_fusion_comparison,
#     show_all,
# )

# from src.visualization.dashboard import show_dashboard
# from src.visualization.style import (
#     apply_style,
#     GNSS_COLOR,
#     RAIM_COLOR,
#     KALMAN_COLOR,
#     INS_COLOR,
#     FUSION_COLOR,
# )


# def print_header():
#     print()
#     print("=" * 70)
#     print("        GNSS / INS Navigation Simulator")
#     print("=" * 70)
#     print()
#     print("Chaîne exécutée :")
#     print("  1. Trajectoire vraie")
#     print("  2. Constellation multi-GNSS")
#     print("  3. Pseudodistances bruitées")
#     print("  4. Gauss-Newton")
#     print("  5. RAIM / FDE")
#     print("  6. Kalman GNSS")
#     print("  7. INS Strapdown")
#     print("  8. Fusion GNSS/INS")
#     print()


# def print_summary(
#     trajectory,
#     gnss,
#     raim,
#     kalman,
#     ins,
#     fusion,
# ):
#     print()
#     print("=" * 70)
#     print("Résumé final")
#     print("=" * 70)

#     print()
#     print("Constellation")
#     print("-" * 70)
#     print(f"Configuration        : {gnss['configuration']}")
#     print(f"Nombre satellites    : {gnss['satellites'].shape[0]}")
#     print(f"PDOP moyen           : {gnss['dop_mean']['PDOP']:.3f}")
#     print(f"HDOP moyen           : {gnss['dop_mean']['HDOP']:.3f}")
#     print(f"VDOP moyen           : {gnss['dop_mean']['VDOP']:.3f}")

#     print()
#     print("GNSS")
#     print("-" * 70)
#     print(f"Gauss-Newton nominal : {gnss['rmse']:.3f} m")
#     print(f"GNSS sans RAIM       : {raim['rmse_raw']:.3f} m")
#     print(f"GNSS avec RAIM       : {raim['rmse_raim']:.3f} m")
#     print(f"Kalman GNSS          : {kalman['rmse']:.3f} m")

#     print()
#     print("RAIM / FDE")
#     print("-" * 70)
#     print(f"Détections           : {raim['n_detections']} / {len(trajectory['t'])}")
#     print(f"Satellite fautif     : {raim['fault_satellite'] + 1}")

#     if raim["most_common_excluded"] is not None:
#         print(f"Satellite exclu      : {raim['most_common_excluded'] + 1}")
#     else:
#         print("Satellite exclu      : Aucun")

#     print()
#     print("INS / Fusion")
#     print("-" * 70)

#     for scenario_name in ins:
#         print()
#         print(f"Scénario             : {ins[scenario_name]['label']}")
#         print(f"INS Strapdown        : {ins[scenario_name]['rmse']:.3f} m")
#         print(f"Fusion GNSS/INS      : {fusion[scenario_name]['rmse']:.3f} m")

#     print()
#     print("=" * 70)


# def main():

#     apply_style()
#     print_header()

#     # ==========================================================
#     # 1. Trajectoire vraie
#     # ==========================================================

#     trajectory = run_trajectory_pipeline(
#         duration=240.0,
#         dt=0.1,
#     )

#     # ==========================================================
#     # 2. GNSS nominal
#     # ==========================================================

#     gnss = run_gnss_pipeline(
#         trajectory=trajectory,
#         gps=8,
#         galileo=6,
#         glonass=0,
#         beidou=0,
#         seed=42,
#         pseudorange_sigma=2.0,
#         multipath_omega=0.05,
#     )

#     # ==========================================================
#     # 3. RAIM / FDE
#     # ==========================================================

#     raim = run_raim_pipeline(
#         trajectory=trajectory,
#         gnss=gnss,
#         fault_satellite=2,
#         fault_bias=30.0,
#         sigma=2.0,
#         pfa=1e-3,
#     )

#     # ==========================================================
#     # 4. Kalman GNSS
#     # ==========================================================

#     kalman = run_kalman_pipeline(
#         trajectory=trajectory,
#         raim=raim,
#         process_noise=0.1,
#         measurement_noise=25.0,
#     )

#     # ==========================================================
#     # 5. INS Strapdown
#     # ==========================================================

#     ins = run_ins_pipeline(
#         trajectory=trajectory,
#         seed=42,
#     )

#     # ==========================================================
#     # 6. Fusion GNSS / INS
#     # ==========================================================

#     fusion = run_fusion_pipeline(
#         trajectory=trajectory,
#         kalman=kalman,
#         ins=ins,
#     )

#     # ==========================================================
#     # 7. Résumé terminal
#     # ==========================================================

#     print_summary(
#         trajectory=trajectory,
#         gnss=gnss,
#         raim=raim,
#         kalman=kalman,
#         ins=ins,
#         fusion=fusion,
#     )

#     # ==========================================================
#     # 8. Visualisations principales
#     # ==========================================================

#     true_position = trajectory["position"]
#     t = trajectory["t"]

#     animate_true_trajectory(
#         trajectory=true_position,
#         interval=10,
#     )

#     animate_constellation(
#         trajectory=true_position,
#         satellites=gnss["satellites"],
#         interval=10,
#     )

#     plot_trajectory(
#         true_position=true_position,
#         estimated_position=raim["estimated_raw"],
#         title="Trajectoire vraie vs GNSS sans RAIM",
#         label="Gauss-Newton sans RAIM",
#         color=GNSS_COLOR,
#     )

#     plot_trajectory(
#         true_position=true_position,
#         estimated_position=raim["estimated_raim"],
#         title="Trajectoire vraie vs GNSS protégé RAIM",
#         label="GNSS avec RAIM/FDE",
#         color=RAIM_COLOR,
#     )

#     plot_trajectory(
#         true_position=true_position,
#         estimated_position=kalman["estimated_positions"],
#         title="Trajectoire vraie vs Kalman GNSS",
#         label="Kalman GNSS",
#         color=KALMAN_COLOR,
#     )

#     plot_position_error(
#         t=t,
#         error=raim["error_raw"],
#         title="Erreur GNSS sans RAIM",
#         color=GNSS_COLOR,
#     )

#     plot_position_error(
#         t=t,
#         error=raim["error_raim"],
#         title="Erreur GNSS avec RAIM/FDE",
#         color=RAIM_COLOR,
#     )

#     plot_raim_statistics(
#         t=t,
#         statistic=raim["statistics"],
#         threshold=np.mean(raim["thresholds"]),
#     )

#     plot_fusion_comparison(
#         t=t,
#         gnss_error=kalman["error"],
#         ins_error=ins["noisy"]["error"],
#         fusion_error=fusion["noisy"]["error"],
#     )

#     show_all()

#     # ==========================================================
#     # 9. Dashboard final
#     # ==========================================================

#     excluded_satellite = None

#     if raim["most_common_excluded"] is not None:
#         excluded_satellite = raim["most_common_excluded"] + 1

#     show_dashboard(
#         configuration=gnss["configuration"],
#         n_satellites=gnss["satellites"].shape[0],
#         pdop=gnss["dop_mean"]["PDOP"],
#         hdop=gnss["dop_mean"]["HDOP"],
#         vdop=gnss["dop_mean"]["VDOP"],
#         gn_rmse=raim["rmse_raw"],
#         raim_rmse=raim["rmse_raim"],
#         kalman_rmse=kalman["rmse"],
#         ins_nominal_rmse=ins["nominal"]["rmse"],
#         fusion_nominal_rmse=fusion["nominal"]["rmse"],
#         ins_noisy_rmse=ins["noisy"]["rmse"],
#         fusion_noisy_rmse=fusion["noisy"]["rmse"],
#         raim_detected=raim["n_detections"] > 0,
#         excluded_satellite=excluded_satellite,
#     )


# if __name__ == "__main__":
#     main()