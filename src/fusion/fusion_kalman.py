"""
Filtre de fusion GNSS / INS basé sur une mécanisation strapdown.

La mécanisation inertielle fournit une prédiction position/vitesse.
Le GNSS fournit une correction de position lorsque le signal est disponible.

État :
    x = [px, py, pz, vx, vy, vz]^T
"""

import numpy as np


class GNSSINSKalman:
    """
    Filtre de Kalman de fusion GNSS/INS.

    Entrée INS :
        position INS
        vitesse INS

    Mesure GNSS :
        position GNSS
    """

    def __init__(
        self,
        measurement_noise=25.0,
        process_noise=0.1,
        initial_covariance=50.0,
    ):
        self.n = 6

        self.x = np.zeros(6)

        self.P = np.eye(self.n) * initial_covariance
        self.Q = np.eye(self.n) * process_noise

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.R = np.eye(3) * measurement_noise

    def initialize(self, position, velocity):
        """
        Initialise l'état du filtre.
        """

        self.x[:3] = position
        self.x[3:] = velocity

    def predict(self, ins_position, ins_velocity):
        """
        Prédiction à partir de la solution INS strapdown.

        Parameters
        ----------
        ins_position : ndarray (3,)
            Position prédite par l'INS.

        ins_velocity : ndarray (3,)
            Vitesse prédite par l'INS.
        """

        self.x[:3] = ins_position
        self.x[3:] = ins_velocity

        self.P = self.P + self.Q

        return self.x.copy()

    def update(self, gnss_position):
        """
        Correction avec une position GNSS.
        """

        gnss_position = np.asarray(gnss_position, dtype=float)

        innovation = gnss_position - self.H @ self.x

        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation

        I = np.eye(self.n)
        self.P = (I - K @ self.H) @ self.P

        return self.x.copy()

    def position(self):
        return self.x[:3].copy()

    def velocity(self):
        return self.x[3:].copy()

    def state(self):
        return self.x.copy()