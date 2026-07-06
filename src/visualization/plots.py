"""
plots.py

Toutes les figures statiques du projet.
"""

import matplotlib.pyplot as plt
import numpy as np

from src.visualization.style import (
    TRUE_COLOR,
    GNSS_COLOR,
    RAIM_COLOR,
    KALMAN_COLOR,
    INS_COLOR,
    FUSION_COLOR,
)


def plot_trajectory(
    true_position,
    estimated_position,
    title="Trajectoire",
    label="Estimation",
    color=GNSS_COLOR,
):

    fig = plt.figure(figsize=(10, 8))

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    ax.plot(
        true_position[:, 0],
        true_position[:, 1],
        true_position[:, 2],
        color=TRUE_COLOR,
        linewidth=3,
        label="Trajectoire vraie",
    )

    ax.plot(
        estimated_position[:, 0],
        estimated_position[:, 1],
        estimated_position[:, 2],
        color=color,
        linewidth=2,
        label=label,
    )

    ax.set_title(title)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.legend()

    plt.tight_layout()


def plot_position_error(
    t,
    error,
    title,
    color,
):

    plt.figure(figsize=(10, 5))

    plt.plot(
        t,
        error,
        color=color,
        linewidth=2,
    )

    plt.grid(True)

    plt.xlabel("Temps (s)")

    plt.ylabel("Erreur (m)")

    plt.title(title)

    plt.tight_layout()


def plot_raim_statistics(
    t,
    statistic,
    threshold,
):

    plt.figure(figsize=(10, 5))

    plt.plot(
        t,
        statistic,
        linewidth=2,
    )

    plt.axhline(
        threshold,
        linestyle="--",
        color="red",
        label="Seuil χ²",
    )

    plt.xlabel("Temps (s)")

    plt.ylabel("Statistique")

    plt.title("Statistique RAIM")

    plt.grid(True)

    plt.legend()

    plt.tight_layout()


def plot_dop(
    t,
    pdop,
    hdop,
    vdop,
):

    plt.figure(figsize=(10, 5))

    plt.plot(
        t,
        pdop,
        label="PDOP",
    )

    plt.plot(
        t,
        hdop,
        label="HDOP",
    )

    plt.plot(
        t,
        vdop,
        label="VDOP",
    )

    plt.grid(True)

    plt.legend()

    plt.xlabel("Temps (s)")

    plt.ylabel("DOP")

    plt.title("Evolution des indicateurs DOP")

    plt.tight_layout()


def plot_fusion_comparison(
    t,
    gnss_error,
    ins_error,
    fusion_error,
):

    plt.figure(figsize=(10, 5))

    plt.plot(
        t,
        gnss_error,
        color=GNSS_COLOR,
        label="GNSS",
    )

    plt.plot(
        t,
        ins_error,
        color=INS_COLOR,
        label="INS",
    )

    plt.plot(
        t,
        fusion_error,
        color=FUSION_COLOR,
        linewidth=3,
        label="Fusion",
    )

    plt.grid(True)

    plt.legend()

    plt.xlabel("Temps (s)")

    plt.ylabel("Erreur (m)")

    plt.title("Comparaison des erreurs")

    plt.tight_layout()


def show_all():

    plt.show()


# def plot_final_navigation_summary(
#     t,
#     true_position,
#     gnss_position,
#     raim_position,
#     kalman_position,
#     ins_position,
#     fusion_position,
#     gnss_error,
#     raim_error,
#     kalman_error,
#     ins_error,
#     fusion_error,
# ):
#     """
#     Figure finale de synthèse :
#     - trajectoires 3D en haut ;
#     - erreurs 3D en bas.
#     """

#     import matplotlib.pyplot as plt

#     from src.visualization.style import (
#         TRUE_COLOR,
#         GNSS_COLOR,
#         RAIM_COLOR,
#         KALMAN_COLOR,
#         INS_COLOR,
#         FUSION_COLOR,
#     )

#     fig = plt.figure(figsize=(14, 10))

#     ax3d = fig.add_subplot(211, projection="3d")

#     ax3d.plot(
#         true_position[:, 0],
#         true_position[:, 1],
#         true_position[:, 2],
#         color=TRUE_COLOR,
#         linewidth=3,
#         label="Trajectoire vraie",
#     )

#     ax3d.plot(
#         gnss_position[:, 0],
#         gnss_position[:, 1],
#         gnss_position[:, 2],
#         color=GNSS_COLOR,
#         alpha=0.35,
#         label="GNSS sans RAIM",
#     )

#     ax3d.plot(
#         raim_position[:, 0],
#         raim_position[:, 1],
#         raim_position[:, 2],
#         color=RAIM_COLOR,
#         alpha=0.65,
#         label="GNSS + RAIM/FDE",
#     )

#     ax3d.plot(
#         kalman_position[:, 0],
#         kalman_position[:, 1],
#         kalman_position[:, 2],
#         color=KALMAN_COLOR,
#         linewidth=2,
#         label="Kalman GNSS",
#     )

#     ax3d.plot(
#         ins_position[:, 0],
#         ins_position[:, 1],
#         ins_position[:, 2],
#         color=INS_COLOR,
#         alpha=0.6,
#         label="INS Strapdown",
#     )

#     ax3d.plot(
#         fusion_position[:, 0],
#         fusion_position[:, 1],
#         fusion_position[:, 2],
#         color=FUSION_COLOR,
#         linewidth=3,
#         label="Fusion GNSS/INS",
#     )

#     ax3d.set_title(
#         "Synthèse trajectoires : GNSS / RAIM / Kalman / INS / Fusion",
#         fontsize=15,
#         weight="bold",
#     )

#     ax3d.set_xlabel("X (m)")
#     ax3d.set_ylabel("Y (m)")
#     ax3d.set_zlabel("Z (m)")
#     ax3d.view_init(elev=25, azim=-60)
#     ax3d.legend(loc="upper right")

#     ax_error = fig.add_subplot(212)

#     ax_error.plot(
#         t,
#         gnss_error,
#         color=GNSS_COLOR,
#         alpha=0.45,
#         label="GNSS sans RAIM",
#     )

#     ax_error.plot(
#         t,
#         raim_error,
#         color=RAIM_COLOR,
#         linewidth=2,
#         label="GNSS + RAIM/FDE",
#     )

#     ax_error.plot(
#         t,
#         kalman_error,
#         color=KALMAN_COLOR,
#         linewidth=2,
#         label="Kalman GNSS",
#     )

#     ax_error.plot(
#         t,
#         ins_error,
#         color=INS_COLOR,
#         alpha=0.6,
#         label="INS Strapdown",
#     )

#     ax_error.plot(
#         t,
#         fusion_error,
#         color=FUSION_COLOR,
#         linewidth=3,
#         label="Fusion GNSS/INS",
#     )

#     ax_error.set_title(
#         "Erreur de position 3D",
#         fontsize=14,
#         weight="bold",
#     )

#     ax_error.set_xlabel("Temps (s)")
#     ax_error.set_ylabel("Erreur 3D (m)")
#     ax_error.grid(True)
#     ax_error.legend(loc="upper left")

#     plt.tight_layout()


def plot_final_navigation_summary(
    t,
    true_position,
    gnss_position,
    raim_position,
    kalman_position,
    ins_nominal_position,
    fusion_nominal_position,
    ins_noisy_position,
    fusion_noisy_position,
    gnss_error,
    raim_error,
    kalman_error,
    ins_nominal_error,
    fusion_nominal_error,
    ins_noisy_error,
    fusion_noisy_error,
):
    import matplotlib.pyplot as plt

    from src.visualization.style import (
        TRUE_COLOR,
        GNSS_COLOR,
        RAIM_COLOR,
        KALMAN_COLOR,
        INS_COLOR,
        FUSION_COLOR,
    )

    fig = plt.figure(figsize=(15, 11))

    ax3d = fig.add_subplot(211, projection="3d")

    ax3d.plot(
        true_position[:, 0],
        true_position[:, 1],
        true_position[:, 2],
        color=TRUE_COLOR,
        linewidth=5,
        label="Trajectoire vraie",
    )

    ax3d.plot(
        gnss_position[:, 0],
        gnss_position[:, 1],
        gnss_position[:, 2],
        color=GNSS_COLOR,
        alpha=0.30,
        linewidth=1.5,
        label="GNSS sans RAIM",
    )

    ax3d.plot(
        raim_position[:, 0],
        raim_position[:, 1],
        raim_position[:, 2],
        color=RAIM_COLOR,
        alpha=0.55,
        linewidth=1.8,
        label="GNSS + RAIM/FDE",
    )

    ax3d.plot(
        kalman_position[:, 0],
        kalman_position[:, 1],
        kalman_position[:, 2],
        color=KALMAN_COLOR,
        linewidth=2,
        label="Kalman GNSS",
    )

    ax3d.plot(
        ins_nominal_position[:, 0],
        ins_nominal_position[:, 1],
        ins_nominal_position[:, 2],
        color=INS_COLOR,
        alpha=0.45,
        linewidth=2,
        label="INS nominale",
    )

    ax3d.plot(
        fusion_nominal_position[:, 0],
        fusion_nominal_position[:, 1],
        fusion_nominal_position[:, 2],
        color=FUSION_COLOR,
        linewidth=4,
        label="Fusion nominale",
    )

    ax3d.plot(
        ins_noisy_position[:, 0],
        ins_noisy_position[:, 1],
        ins_noisy_position[:, 2],
        color="black",
        alpha=0.35,
        linewidth=2,
        label="INS bruitée",
    )

    ax3d.plot(
        fusion_noisy_position[:, 0],
        fusion_noisy_position[:, 1],
        fusion_noisy_position[:, 2],
        color="gray",
        alpha=0.55,
        linewidth=2,
        label="Fusion bruitée",
    )

    ax3d.set_title(
        "Synthèse trajectoires : GNSS / RAIM / Kalman / INS / Fusion",
        fontsize=15,
        weight="bold",
    )

    ax3d.set_xlabel("X (m)")
    ax3d.set_ylabel("Y (m)")
    ax3d.set_zlabel("Z (m)")
    ax3d.view_init(elev=25, azim=-60)
    ax3d.legend(loc="upper right")

    ax_error = fig.add_subplot(212)

    ax_error.plot(t, gnss_error, color=GNSS_COLOR, alpha=0.35, label="GNSS sans RAIM")
    ax_error.plot(t, raim_error, color=RAIM_COLOR, linewidth=2, label="GNSS + RAIM/FDE")
    ax_error.plot(t, kalman_error, color=KALMAN_COLOR, linewidth=2, label="Kalman GNSS")

    ax_error.plot(
        t,
        ins_nominal_error,
        color=INS_COLOR,
        alpha=0.55,
        linewidth=2,
        label="INS nominale",
    )

    ax_error.plot(
        t,
        fusion_nominal_error,
        color=FUSION_COLOR,
        linewidth=3,
        label="Fusion nominale",
    )

    ax_error.plot(
        t,
        ins_noisy_error,
        color="black",
        alpha=0.45,
        linewidth=2,
        label="INS bruitée",
    )

    ax_error.plot(
        t,
        fusion_noisy_error,
        color="gray",
        alpha=0.70,
        linewidth=2,
        label="Fusion bruitée",
    )

    ax_error.set_title("Erreur de position 3D", fontsize=14, weight="bold")
    ax_error.set_xlabel("Temps (s)")
    ax_error.set_ylabel("Erreur 3D (m)")
    ax_error.grid(True)
    ax_error.legend(loc="upper left")

    plt.tight_layout()

    plt.savefig(
        "results/figures/final_summary.png",
        dpi=300,
        bbox_inches="tight",
    )
    plt.show()