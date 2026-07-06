"""
trajectory_pipeline.py

Pipeline de génération de la trajectoire de référence.
"""

from src.simulation.trajectory import generate_drone_trajectory
from src.simulation.kinematics import compute_kinematic_state


def run_trajectory_pipeline(
    duration=240.0,
    dt=0.1,
):
    """
    Génère la trajectoire vraie et l'état cinématique complet.
    """

    t, position, velocity, acceleration = generate_drone_trajectory(
        duration=duration,
        dt=dt,
    )

    kinematic_state = compute_kinematic_state(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        dt=dt,
    )

    return {
        "dt": dt,
        "t": t,
        "position": position,
        "velocity": velocity,
        "acceleration": acceleration,
        "kinematic_state": kinematic_state,
        "specific_force_body": kinematic_state["specific_force_body"],
        "angular_rates": kinematic_state["angular_rates"],
        "quaternions": kinematic_state["quaternions"],
        "initial_position": position[0],
        "initial_velocity": velocity[0],
        "initial_attitude": kinematic_state["quaternions"][0],
    }