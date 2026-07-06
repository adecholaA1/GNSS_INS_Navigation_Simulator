"""
scene.py

Création des objets graphiques utilisés par les animations
du simulateur GNSS / INS.

Aucun calcul de navigation n'est effectué ici.
Le module ne fait que créer les éléments visuels.
"""

import numpy as np


def create_satellites(ax, satellites):
    """
    Affiche les satellites.

    Returns
    -------
    Path3DCollection
    """

    return ax.scatter(
        satellites[:, 0],
        satellites[:, 1],
        satellites[:, 2],
        marker="^",
        s=80,
        color="gold",
        edgecolors="black",
        label="Satellites",
        zorder=10,
    )


def create_drone(ax):
    """
    Crée le drone.

    Returns
    -------
    Path3DCollection
    """

    return ax.scatter(
        [],
        [],
        [],
        s=90,
        color="red",
        edgecolors="black",
        label="Drone",
        zorder=20,
    )


def create_true_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=2.5,
        color="limegreen",
        label="Trajectoire vraie",
    )

    return line


def create_gnss_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=2,
        color="crimson",
        label="GNSS",
    )

    return line


def create_raim_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=2,
        color="darkorange",
        label="RAIM",
    )

    return line


def create_kalman_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=2,
        color="royalblue",
        label="Kalman",
    )

    return line


def create_ins_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=2,
        color="purple",
        label="INS",
    )

    return line


def create_fusion_trajectory(ax):

    line, = ax.plot(
        [],
        [],
        [],
        linewidth=3,
        color="cyan",
        label="Fusion GNSS/INS",
    )

    return line


def create_visibility_lines(ax, satellites):
    """
    Crée les lignes de visée Satellite -> Drone.
    """

    lines = []

    for _ in satellites:

        line, = ax.plot(
            [],
            [],
            [],
            color="gray",
            alpha=0.25,
            linewidth=0.8,
        )

        lines.append(line)

    return lines


def create_fault_marker(ax):
    """
    Texte dynamique utilisé pendant RAIM.
    """

    return ax.text2D(
        0.02,
        0.95,
        "",
        transform=ax.transAxes,
        fontsize=13,
        color="red",
        weight="bold",
    )


def update_drone(drone, position):
    """
    Déplace le drone.
    """

    drone._offsets3d = (
        [position[0]],
        [position[1]],
        [position[2]],
    )


def update_line(line, trajectory, frame):
    """
    Met à jour une trajectoire.
    """

    line.set_data(
        trajectory[:frame + 1, 0],
        trajectory[:frame + 1, 1],
    )

    line.set_3d_properties(
        trajectory[:frame + 1, 2],
    )


def update_visibility(lines, satellites, drone_position):
    """
    Met à jour les lignes de visée.
    """

    for i, sat in enumerate(satellites):

        lines[i].set_data(
            [sat[0], drone_position[0]],
            [sat[1], drone_position[1]],
        )

        lines[i].set_3d_properties(
            [sat[2], drone_position[2]],
        )