"""
style.py

Style graphique commun à toutes les visualisations
du simulateur GNSS/INS.
"""

import matplotlib.pyplot as plt


# ==========================================================
# Couleurs
# ==========================================================

TRUE_COLOR = "#e74c3c"

GNSS_COLOR =  "#2ecc71"

RAIM_COLOR = "#f39c12"

KALMAN_COLOR = "#3498db"

INS_COLOR = "#9b59b6"

FUSION_COLOR = "#1abc9c"

SATELLITE_COLOR = "#f1c40f"

FAULT_COLOR = "#c0392b"

VISIBILITY_COLOR = "#95a5a6"

EARTH_COLOR = "#bdc3c7"


# ==========================================================
# Tailles
# ==========================================================

LINEWIDTH = 2.5

MARKER_SIZE = 70

TITLE_SIZE = 16

LABEL_SIZE = 12

LEGEND_SIZE = 11


# ==========================================================
# Figure
# ==========================================================

FIGSIZE_3D = (11, 9)

FIGSIZE_2D = (10, 6)


# ==========================================================
# Police
# ==========================================================

FONT = "DejaVu Sans"


# ==========================================================
# Application du thème
# ==========================================================

def apply_style():

    plt.rcParams["font.family"] = FONT

    plt.rcParams["axes.grid"] = True

    plt.rcParams["grid.alpha"] = 0.35

    plt.rcParams["axes.titlesize"] = TITLE_SIZE

    plt.rcParams["axes.labelsize"] = LABEL_SIZE

    plt.rcParams["legend.fontsize"] = LEGEND_SIZE

    plt.rcParams["figure.figsize"] = FIGSIZE_2D

    plt.rcParams["lines.linewidth"] = LINEWIDTH