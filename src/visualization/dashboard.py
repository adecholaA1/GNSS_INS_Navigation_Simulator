"""
dashboard.py

Affichage du tableau de synthèse du simulateur GNSS/INS.
"""

import matplotlib.pyplot as plt


def show_dashboard(
    configuration,
    n_satellites,
    pdop,
    hdop,
    vdop,
    gn_rmse,
    raim_rmse,
    kalman_rmse,
    ins_nominal_rmse,
    fusion_nominal_rmse,
    ins_noisy_rmse,
    fusion_noisy_rmse,
    raim_detected=True,
    excluded_satellite=None,
):
    """
    Affiche le tableau de bord final.
    """

    fig = plt.figure(figsize=(12, 8))

    ax = fig.add_subplot(111)

    ax.axis("off")

    title = (
        "GNSS / INS Navigation Simulator\n"
        "Résultats de la simulation"
    )

    ax.set_title(
        title,
        fontsize=18,
        weight="bold",
        pad=20,
    )

    text = f"""
Configuration GNSS
------------------------------
{configuration}

Nombre de satellites
------------------------------
{n_satellites}

Qualité de géométrie
------------------------------
PDOP : {pdop:.3f}
HDOP : {hdop:.3f}
VDOP : {vdop:.3f}

RAIM / FDE
------------------------------
Détection : {'Oui' if raim_detected else 'Non'}
Satellite exclu : {excluded_satellite}

Précision GNSS
------------------------------
Gauss-Newton : {gn_rmse:.3f} m
RAIM         : {raim_rmse:.3f} m
Kalman       : {kalman_rmse:.3f} m

Navigation inertielle
------------------------------
INS nominale        : {ins_nominal_rmse:.3f} m
Fusion nominale     : {fusion_nominal_rmse:.3f} m

INS bruitée         : {ins_noisy_rmse:.3f} m
Fusion bruitée      : {fusion_noisy_rmse:.3f} m
"""

    ax.text(
        0.05,
        0.95,
        text,
        fontsize=13,
        va="top",
        family="monospace",
    )

    plt.tight_layout()
    plt.show()