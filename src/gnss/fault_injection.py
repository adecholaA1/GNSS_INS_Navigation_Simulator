"""
Injection de défauts GNSS.

Ce module permet de créer différents scénarios de dégradation des
pseudodistances afin de tester les algorithmes RAIM, Kalman,
GNSS/INS et les mécanismes d'intégrité.

Toutes les fonctions retournent une copie des pseudodistances.
"""

import numpy as np


def inject_constant_bias(
    pseudoranges,
    satellite_index,
    bias,
):
    """
    Ajoute un biais constant sur un satellite.
    """

    corrupted = pseudoranges.copy()

    corrupted[:, satellite_index] += bias

    return corrupted


def inject_time_window_bias(
    pseudoranges,
    satellite_index,
    start_time,
    end_time,
    bias,
    dt,
):
    """
    Ajoute un biais uniquement sur une fenêtre temporelle.
    """

    corrupted = pseudoranges.copy()

    start = int(start_time / dt)
    stop = int(end_time / dt)

    corrupted[start:stop, satellite_index] += bias

    return corrupted


def inject_ramp_bias(
    pseudoranges,
    satellite_index,
    start_time,
    end_time,
    final_bias,
    dt,
):
    """
    Biais qui augmente progressivement.
    """

    corrupted = pseudoranges.copy()

    start = int(start_time / dt)
    stop = int(end_time / dt)

    ramp = np.linspace(
        0.0,
        final_bias,
        stop - start,
    )

    corrupted[start:stop, satellite_index] += ramp

    return corrupted


def inject_random_outliers(
    pseudoranges,
    probability,
    amplitude,
    seed=None,
):
    """
    Injecte des valeurs aberrantes aléatoires.
    """

    rng = np.random.default_rng(seed)

    corrupted = pseudoranges.copy()

    mask = rng.random(corrupted.shape) < probability

    jumps = rng.uniform(
        -amplitude,
        amplitude,
        corrupted.shape,
    )

    corrupted[mask] += jumps[mask]

    return corrupted


def inject_cycle_slip(
    pseudoranges,
    satellite_index,
    epoch,
    jump,
):
    """
    Simule un cycle slip.

    Toutes les mesures après l'époque considérée sont décalées.
    """

    corrupted = pseudoranges.copy()

    corrupted[epoch:, satellite_index] += jump

    return corrupted


def inject_satellite_dropout(
    pseudoranges,
    satellite_index,
    start_time,
    end_time,
    dt,
):
    """
    Simule la perte d'un satellite.

    Les mesures deviennent invalides (NaN).
    """

    corrupted = pseudoranges.copy()

    start = int(start_time / dt)
    stop = int(end_time / dt)

    corrupted[start:stop, satellite_index] = np.nan

    return corrupted


def inject_clock_jump(
    pseudoranges,
    jump,
    epoch,
):
    """
    Simule un saut d'horloge récepteur.

    Toutes les pseudodistances sont affectées.
    """

    corrupted = pseudoranges.copy()

    corrupted[epoch:] += jump

    return corrupted


def inject_multipath_burst(
    pseudoranges,
    satellite_index,
    start_time,
    end_time,
    amplitude,
    frequency,
    dt,
):
    """
    Simule un multipath localisé.

    Une oscillation sinusoïdale est appliquée
    sur une fenêtre temporelle.
    """

    corrupted = pseudoranges.copy()

    start = int(start_time / dt)
    stop = int(end_time / dt)

    t = np.arange(stop - start) * dt

    burst = amplitude * np.sin(
        2.0 * np.pi * frequency * t
    )

    corrupted[start:stop, satellite_index] += burst

    return corrupted