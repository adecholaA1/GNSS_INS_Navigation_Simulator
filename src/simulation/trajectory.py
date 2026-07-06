import numpy as np


def _resample_by_distance(points, n_samples):
    """
    Rééchantillonne une trajectoire suivant la distance parcourue.

    Le parcours initial est défini par un ensemble de points dont
    l'espacement est variable (segments rectilignes et virages).

    Un rééchantillonnage uniforme suivant la distance permet de produire
    une trajectoire dont la vitesse reste pratiquement constante.

    Cette étape évite l'apparition d'accélérations artificielles qui
    seraient uniquement dues à la discrétisation de la trajectoire.

    Parameters
    ----------
    points : ndarray (N,3)
        Points décrivant la trajectoire.

    n_samples : int
        Nombre d'échantillons souhaité.

    Returns
    -------
    ndarray (n_samples,3)
        Trajectoire rééchantillonnée.
    """

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)

    cumulative_distance = np.insert(
        np.cumsum(segment_lengths),
        0,
        0.0
    )

    target_distance = np.linspace(
        0.0,
        cumulative_distance[-1],
        n_samples
    )

    x = np.interp(
        target_distance,
        cumulative_distance,
        points[:, 0]
    )

    y = np.interp(
        target_distance,
        cumulative_distance,
        points[:, 1]
    )

    z = np.interp(
        target_distance,
        cumulative_distance,
        points[:, 2]
    )

    return np.column_stack((x, y, z))


def generate_drone_trajectory(
    duration=240.0,
    dt=0.1,
    length=600.0,
    width=300.0,
    altitude=120.0,
    n_passes=4,
    altitude_variation=3.0,
):
    """
    Génère une trajectoire 3D représentative d'une mission
    de cartographie aérienne.

    Le drone suit une trajectoire de type "lawn mower"
    constituée de plusieurs passes parallèles reliées par
    des virages continus.

    La trajectoire est ensuite rééchantillonnée afin de
    conserver une vitesse quasi constante sur l'ensemble
    de la mission.

    Une faible oscillation verticale est ajoutée afin de
    représenter les corrections d'altitude rencontrées
    durant un vol réel.

    Parameters
    ----------
    duration : float
        Durée totale de la mission (s).

    dt : float
        Pas d'échantillonnage (s).

    length : float
        Longueur de la zone cartographiée (m).

    width : float
        Largeur de la zone cartographiée (m).

    altitude : float
        Altitude moyenne du drone (m).

    n_passes : int
        Nombre de passes de cartographie.

    altitude_variation : float
        Amplitude des variations d'altitude (m).

    Returns
    -------
    t : ndarray
        Temps de simulation.

    position : ndarray (N,3)
        Position réelle du drone.

    velocity : ndarray (N,3)
        Vitesse réelle.

    acceleration : ndarray (N,3)
        Accélération réelle.
    """

    # Axe temporel de la simulation

    t = np.arange(
        0.0,
        duration,
        dt
    )

    n_samples = len(t)

    # Construction géométrique de la trajectoire avant
    # rééchantillonnage.

    points = []

    y_lines = np.linspace(
        -width / 2,
        width / 2,
        n_passes
    )

    turn_radius = width / (2 * (n_passes - 1))

    for i in range(n_passes):

        y = y_lines[i]

        # Passe de cartographie.

        if i % 2 == 0:

            x_line = np.linspace(
                0.0,
                length,
                300
            )

        else:

            x_line = np.linspace(
                length,
                0.0,
                300
            )

        y_line = np.full_like(
            x_line,
            y
        )

        for x, yy in zip(x_line, y_line):

            points.append([
                x,
                yy,
                altitude
            ])

        # Virage reliant deux passes successives.

        if i < n_passes - 1:

            y_next = y_lines[i + 1]

            y_center = (y + y_next) / 2

            if i % 2 == 0:

                x_center = length

                theta = np.linspace(
                    -np.pi / 2,
                    np.pi / 2,
                    120
                )

            else:

                x_center = 0.0

                theta = np.linspace(
                    -np.pi / 2,
                    -3 * np.pi / 2,
                    120
                )

            x_turn = (
                x_center
                + turn_radius * np.cos(theta)
            )

            y_turn = (
                y_center
                + turn_radius * np.sin(theta)
            )

            for x, yy in zip(x_turn, y_turn):

                points.append([
                    x,
                    yy,
                    altitude
                ])

    points = np.array(points)

    # Rééchantillonnage afin d'obtenir une vitesse
    # pratiquement constante sur toute la mission.

    position = _resample_by_distance(
        points,
        n_samples
    )

    # Modélisation d'une faible variation d'altitude
    # correspondant aux corrections verticales du pilote
    # automatique.

    position[:, 2] = (
        altitude
        + altitude_variation
        * np.sin(2 * np.pi * t / duration)
    )

    # Calcul des grandeurs cinématiques à partir de
    # la trajectoire réelle.

    velocity = np.gradient(
        position,
        dt,
        axis=0
    )

    acceleration = np.gradient(
        velocity,
        dt,
        axis=0
    )

    return (
        t,
        position,
        velocity,
        acceleration
    )


