"""
Cinématique véhicule pour la simulation IMU.

Ce module transforme une trajectoire vraie en grandeurs exploitables par
une centrale inertielle :

    position, vitesse, accélération
        ↓
    roll, pitch, yaw
        ↓
    quaternion
        ↓
    matrices de rotation
        ↓
    force spécifique dans le repère body
        ↓
    vitesses angulaires gyroscope

Convention :
    - quaternion : q = [w, x, y, z]
    - repère navigation : z vers le haut
    - repère body : repère lié au drone
"""

import numpy as np

from src.ins.gravity import gravity_vector


def compute_yaw(velocity):
    vx = velocity[:, 0]
    vy = velocity[:, 1]
    return np.unwrap(np.arctan2(vy, vx))


def compute_pitch(velocity):
    horizontal_speed = np.sqrt(
        velocity[:, 0] ** 2
        + velocity[:, 1] ** 2
    )

    return np.arctan2(
        -velocity[:, 2],
        horizontal_speed
    )


def compute_roll(n_samples):
    return np.zeros(n_samples)


def euler_to_quaternion(roll, pitch, yaw):
    cr = np.cos(roll / 2.0)
    sr = np.sin(roll / 2.0)

    cp = np.cos(pitch / 2.0)
    sp = np.sin(pitch / 2.0)

    cy = np.cos(yaw / 2.0)
    sy = np.sin(yaw / 2.0)

    q = np.column_stack(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        )
    )

    norms = np.linalg.norm(q, axis=1)

    return q / norms[:, None]


def quaternion_to_rotation_matrix(q):
    w, x, y, z = q

    return np.array(
        [
            [
                1.0 - 2.0 * (y**2 + z**2),
                2.0 * (x*y - z*w),
                2.0 * (x*z + y*w),
            ],
            [
                2.0 * (x*y + z*w),
                1.0 - 2.0 * (x**2 + z**2),
                2.0 * (y*z - x*w),
            ],
            [
                2.0 * (x*z - y*w),
                2.0 * (y*z + x*w),
                1.0 - 2.0 * (x**2 + y**2),
            ],
        ],
        dtype=float
    )


def compute_rotation_matrices(quaternions):
    n_samples = quaternions.shape[0]

    rotation_matrices = np.zeros(
        (n_samples, 3, 3),
        dtype=float
    )

    for k in range(n_samples):
        rotation_matrices[k] = quaternion_to_rotation_matrix(
            quaternions[k]
        )

    return rotation_matrices


def compute_angular_rates(roll, pitch, yaw, dt):
    wx = np.gradient(roll, dt)
    wy = np.gradient(pitch, dt)
    wz = np.gradient(yaw, dt)

    return np.column_stack((wx, wy, wz))


def compute_specific_force_body(acceleration_navigation, rotation_matrices):
    """
    Calcule la force spécifique mesurée par un accéléromètre.

    Modèle :
        f_nav = a_nav - g_nav
        f_body = R_nav_body f_nav

    Comme les matrices stockées sont body → navigation,
    on utilise leur transposée pour obtenir navigation → body.
    """

    g_nav = gravity_vector()

    n_samples = acceleration_navigation.shape[0]

    specific_force_body = np.zeros(
        (n_samples, 3),
        dtype=float
    )

    for k in range(n_samples):
        specific_force_navigation = (
            acceleration_navigation[k]
            - g_nav
        )

        specific_force_body[k] = (
            rotation_matrices[k].T
            @ specific_force_navigation
        )

    return specific_force_body


def compute_kinematic_state(position, velocity, acceleration, dt):
    """
    Calcule l'état cinématique complet du véhicule.

    Returns
    -------
    dict
        Grandeurs cinématiques nécessaires à la simulation IMU.
    """

    n_samples = position.shape[0]

    roll = compute_roll(n_samples)
    pitch = compute_pitch(velocity)
    yaw = compute_yaw(velocity)

    quaternions = euler_to_quaternion(
        roll,
        pitch,
        yaw
    )

    rotation_matrices = compute_rotation_matrices(
        quaternions
    )

    angular_rates = compute_angular_rates(
        roll,
        pitch,
        yaw,
        dt
    )

    specific_force_body = compute_specific_force_body(
        acceleration,
        rotation_matrices
    )

    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "quaternions": quaternions,
        "rotation_matrices": rotation_matrices,
        "angular_rates": angular_rates,
        "specific_force_body": specific_force_body,
    }