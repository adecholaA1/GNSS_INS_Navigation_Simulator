"""
Filtre de Kalman linéaire adapté au cas GNSS 3D.

Adaptation du formalisme générique :

    x_k = A x_{k-1} + B u_k
    z_k = C x_k

au cas :

    x = [px, py, pz, vx, vy, vz]^T
    z = [px, py, pz]^T
"""

import numpy as np


class KalmanFilter3D:
    """
    Filtre de Kalman linéaire pour lisser une trajectoire GNSS.

    L'entrée du filtre est une suite de positions estimées par GNSS,
    typiquement issues de Gauss-Newton.
    """

    def __init__(
        self,
        dt,
        process_noise=0.5,
        measurement_noise=20.0,
    ):
        self.dt = dt

        # Etat : [px, py, pz, vx, vy, vz]^T
        self.x = np.zeros((6, 1))

        # Matrice d'évolution A : modèle à vitesse constante
        self.A = np.array(
            [
                [1, 0, 0, dt, 0,  0],
                [0, 1, 0, 0,  dt, 0],
                [0, 0, 1, 0,  0,  dt],
                [0, 0, 0, 1,  0,  0],
                [0, 0, 0, 0,  1,  0],
                [0, 0, 0, 0,  0,  1],
            ],
            dtype=float,
        )

        # Pas de commande externe pour ce Kalman GNSS simple
        self.B = np.zeros((6, 1))

        # Matrice de mesure C : on mesure seulement la position
        self.C = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=float,
        )

        # Covariance initiale
        self.P = np.eye(6) * 100.0

        # Bruit du modèle
        self.Q = np.eye(6) * process_noise

        # Bruit de mesure GNSS
        self.R = np.eye(3) * measurement_noise

        # Historiques utiles pour analyse
        self.predicted_states = []
        self.corrected_states = []
        self.predicted_covariances = []
        self.corrected_covariances = []
        self.kalman_gains = []
        self.innovations = []

    def initialize(self, position, velocity=None):
        """
        Initialise l'état du filtre.

        Parameters
        ----------
        position : ndarray (3,)
            Position initiale.
        velocity : ndarray (3,), optional
            Vitesse initiale.
        """

        self.x[0:3, 0] = position

        if velocity is not None:
            self.x[3:6, 0] = velocity

        self.corrected_states.append(self.x.copy())
        self.corrected_covariances.append(self.P.copy())

    def predict(self, u=None):
        """
        Etape de prédiction.

        Parameters
        ----------
        u : ndarray, optional
            Commande externe. Non utilisée ici.
        """

        if u is None:
            u = np.zeros((1, 1))

        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q

        self.predicted_states.append(self.x.copy())
        self.predicted_covariances.append(self.P.copy())

        return self.position()

    def update(self, measurement):
        """
        Etape de correction avec une mesure GNSS.

        Parameters
        ----------
        measurement : ndarray (3,)
            Position GNSS estimée.

        Returns
        -------
        ndarray (3,)
            Position filtrée.
        """

        z = measurement.reshape(3, 1)

        innovation = z - self.C @ self.x
        S = self.C @ self.P @ self.C.T + self.R
        K = self.P @ self.C.T @ np.linalg.inv(S)

        self.x = self.x + K @ innovation
        self.P = (np.eye(6) - K @ self.C) @ self.P

        self.innovations.append(innovation.copy())
        self.kalman_gains.append(K.copy())
        self.corrected_states.append(self.x.copy())
        self.corrected_covariances.append(self.P.copy())

        return self.position()

    def position(self):
        return self.x[0:3, 0]

    def velocity(self):
        return self.x[3:6, 0]

    def state(self):
        return self.x[:, 0]