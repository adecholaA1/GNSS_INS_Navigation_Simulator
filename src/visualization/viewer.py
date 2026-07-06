"""
viewer.py

Visualiseur 3D PyVista pour la démonstration GNSS/INS.
"""

import numpy as np
import pyvista as pv


class NavigationViewer:
    def __init__(self):
        self.plotter = pv.Plotter()
        self.plotter.set_background("white")

    def load(
        self,
        trajectory,
        gnss,
        raim,
        kalman,
        ins,
        fusion,
    ):
        self.trajectory = trajectory
        self.gnss = gnss
        self.raim = raim
        self.kalman = kalman
        self.ins = ins
        self.fusion = fusion

        self.true_position = trajectory["position"]
        self.satellites = gnss["satellites"]

    def setup_scene(self):
        self.plotter.add_axes()
        self.plotter.add_text(
            "GNSS / INS Navigation Simulator",
            position="upper_left",
            font_size=12,
            color="black",
        )

        self.plotter.add_mesh(
            pv.Sphere(radius=6.0),
            color="red",
            name="drone",
        )

        self.plotter.camera_position = "xy"

    def add_static_trajectories(self):
        true_line = pv.Spline(
            self.true_position,
            n_points=len(self.true_position),
        )

        raim_line = pv.Spline(
            self.raim["estimated_raim"],
            n_points=len(self.raim["estimated_raim"]),
        )

        kalman_line = pv.Spline(
            self.kalman["estimated_positions"],
            n_points=len(self.kalman["estimated_positions"]),
        )

        fusion_line = pv.Spline(
            self.fusion["nominal"]["position"],
            n_points=len(self.fusion["nominal"]["position"]),
        )

        self.plotter.add_mesh(
            true_line,
            color="red",
            line_width=5,
            label="Trajectoire vraie",
        )

        self.plotter.add_mesh(
            raim_line,
            color="orange",
            line_width=3,
            label="GNSS + RAIM",
        )

        self.plotter.add_mesh(
            kalman_line,
            color="blue",
            line_width=3,
            label="Kalman GNSS",
        )

        self.plotter.add_mesh(
            fusion_line,
            color="green",
            line_width=4,
            label="Fusion GNSS/INS",
        )

        self.plotter.add_legend()

    def add_satellites(self):
        scale = 1e-5

        for i, sat in enumerate(self.satellites):
            sat_scaled = sat * scale

            sphere = pv.Sphere(
                radius=8.0,
                center=sat_scaled,
            )

            self.plotter.add_mesh(
                sphere,
                color="gold",
                name=f"satellite_{i+1}",
            )

    def run(self):
        self.setup_scene()
        self.add_static_trajectories()
        self.add_satellites()
        self.plotter.show()