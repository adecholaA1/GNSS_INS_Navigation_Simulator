"""
Error-State Kalman Filter pour fusion GNSS / INS.

Etat d'erreur :
    dx = [
        dp_x, dp_y, dp_z,
        dv_x, dv_y, dv_z,
        dtheta_x, dtheta_y, dtheta_z,
        db_a_x, db_a_y, db_a_z,
        db_g_x, db_g_y, db_g_z
    ]

Le filtre estime les erreurs de l'état nominal INS ainsi que les biais
accéléromètre et gyroscope.
"""

import numpy as np


class ErrorStateKalman:
    """
    Error-State Kalman Filter à 15 états.
    """

    def __init__(
        self,
        dt,
        position_noise=25.0,
        process_noise_position=0.01,
        process_noise_velocity=0.05,
        process_noise_attitude=0.001,
        process_noise_accel_bias=1e-4,
        process_noise_gyro_bias=1e-6,
        initial_covariance=10.0,
    ):
        self.dt = dt
        self.n = 15

        self.dx = np.zeros(self.n)
        self.P = np.eye(self.n) * initial_covariance

        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        self.Q = np.diag([
            process_noise_position,
            process_noise_position,
            process_noise_position,
            process_noise_velocity,
            process_noise_velocity,
            process_noise_velocity,
            process_noise_attitude,
            process_noise_attitude,
            process_noise_attitude,
            process_noise_accel_bias,
            process_noise_accel_bias,
            process_noise_accel_bias,
            process_noise_gyro_bias,
            process_noise_gyro_bias,
            process_noise_gyro_bias,
        ])

        self.R = np.eye(3) * position_noise

        self.H = np.zeros((3, self.n))
        self.H[:, 0:3] = np.eye(3)

    def predict(self):
        """
        Propage la covariance de l'état d'erreur.
        """

        F = np.eye(self.n)

        F[0:3, 3:6] = np.eye(3) * self.dt
        F[3:6, 9:12] = -np.eye(3) * self.dt
        F[6:9, 12:15] = -np.eye(3) * self.dt

        self.P = F @ self.P @ F.T + self.Q

    def update_gnss(self, nominal_position, gnss_position):
        """
        Corrige l'état d'erreur avec une position GNSS.
        """

        nominal_position = np.asarray(nominal_position, dtype=float)
        gnss_position = np.asarray(gnss_position, dtype=float)

        innovation = gnss_position - nominal_position

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.dx = K @ innovation

        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P

        correction_position = self.dx[0:3].copy()
        correction_velocity = self.dx[3:6].copy()
        correction_attitude = self.dx[6:9].copy()

        correction_accel_bias = self.dx[9:12].copy()
        correction_gyro_bias = self.dx[12:15].copy()

        self.accel_bias += correction_accel_bias
        self.gyro_bias += correction_gyro_bias

        corrections = {
            "position": correction_position,
            "velocity": correction_velocity,
            "attitude": correction_attitude,
            "accel_bias": self.accel_bias.copy(),
            "gyro_bias": self.gyro_bias.copy(),
        }

        self.dx[:] = 0.0

        return corrections

    def get_accel_bias(self):
        """
        Retourne le biais accéléromètre estimé.
        """

        return self.accel_bias.copy()

    def get_gyro_bias(self):
        """
        Retourne le biais gyroscope estimé.
        """

        return self.gyro_bias.copy()

    def reset_biases(self):
        """
        Réinitialise les biais estimés.
        """

        self.accel_bias[:] = 0.0
        self.gyro_bias[:] = 0.0