import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import FFMpegWriter


def animate_true_trajectory(
    trajectory,
    interval=20,
):
    """
    Animation de la trajectoire réelle.

    Parameters
    ----------
    trajectory : ndarray (N,3)

    interval : int
        Temps entre deux images (ms).
    """

    fig = plt.figure(figsize=(10, 8))

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    ax.set_title(
        "Trajectoire réelle du drone",
        fontsize=16,
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.set_xlim(
        np.min(trajectory[:, 0]) - 20,
        np.max(trajectory[:, 0]) + 20,
    )

    ax.set_ylim(
        np.min(trajectory[:, 1]) - 20,
        np.max(trajectory[:, 1]) + 20,
    )

    ax.set_zlim(
        np.min(trajectory[:, 2]) - 5,
        np.max(trajectory[:, 2]) + 5,
    )

    ax.grid(True)

    line, = ax.plot(
        [],
        [],
        [],
        lw=2,
        color="royalblue",
        label="Trajectoire",
    )

    point = ax.scatter(
        [],
        [],
        [],
        s=60,
        color="red",
        label="Drone",
    )

    ax.legend()

    def update(frame):

        line.set_data(
            trajectory[:frame, 0],
            trajectory[:frame, 1],
        )

        line.set_3d_properties(
            trajectory[:frame, 2],
        )

        point._offsets3d = (
            [trajectory[frame, 0]],
            [trajectory[frame, 1]],
            [trajectory[frame, 2]],
        )

        return line, point

    animation = FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=interval,
        blit=False,
        repeat=False,
    )

    plt.show()

    return animation



def animate_constellation(
    trajectory,
    satellites,
    interval=20,
):
    """
    Animation de la constellation GNSS et du drone.

    Le drone suit sa trajectoire tandis que les satellites
    restent fixes dans le repère ECEF simplifié.
    """

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")

    ax.set_title("Constellation GNSS", fontsize=16)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    margin = 5e6

    ax.set_xlim(
        np.min(satellites[:,0]) - margin,
        np.max(satellites[:,0]) + margin,
    )

    ax.set_ylim(
        np.min(satellites[:,1]) - margin,
        np.max(satellites[:,1]) + margin,
    )

    ax.set_zlim(
        np.min(satellites[:,2]) - margin,
        np.max(satellites[:,2]) + margin,
    )

    ax.grid(True)

    # Satellites
    ax.scatter(
        satellites[:,0],
        satellites[:,1],
        satellites[:,2],
        s=80,
        color="orange",
        marker="^",
        label="Satellites",
    )

    # Drone
    drone = ax.scatter(
        [],
        [],
        [],
        s=80,
        color="red",
        label="Drone",
    )

    # Trajectoire
    trajectory_line, = ax.plot(
        [],
        [],
        [],
        color="royalblue",
        linewidth=2,
        label="Trajectoire réelle",
    )

    # Lignes de visée
    visibility_lines = []

    for _ in satellites:
        line, = ax.plot(
            [],
            [],
            [],
            color="gray",
            alpha=0.25,
            linewidth=0.8,
        )
        visibility_lines.append(line)

    ax.legend()

    def update(frame):

        p = trajectory[frame]

        drone._offsets3d = (
            [p[0]],
            [p[1]],
            [p[2]],
        )

        trajectory_line.set_data(
            trajectory[:frame+1,0],
            trajectory[:frame+1,1],
        )

        trajectory_line.set_3d_properties(
            trajectory[:frame+1,2],
        )

        for i, sat in enumerate(satellites):

            visibility_lines[i].set_data(
                [sat[0], p[0]],
                [sat[1], p[1]],
            )

            visibility_lines[i].set_3d_properties(
                [sat[2], p[2]]
            )

        return (
            [trajectory_line, drone]
            + visibility_lines
        )

    anim = FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=interval,
        blit=False,
        repeat=False,
    )

    plt.show()

    return anim




def animate_navigation_pipeline(
    true_position,
    satellites,
    gnss_position,
    raim_position,
    kalman_position,
    ins_position,
    fusion_position,
    raim_flags=None,
    excluded_satellites=None,
    interval=10,
    step=5,
):
    """
    Animation unique de la chaîne GNSS/INS complète.
    """

    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    from src.visualization.style import (
        TRUE_COLOR,
        GNSS_COLOR,
        RAIM_COLOR,
        KALMAN_COLOR,
        INS_COLOR,
        FUSION_COLOR,
        SATELLITE_COLOR,
        FAULT_COLOR,
        VISIBILITY_COLOR,
    )

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    ax.set_title(
        "GNSS / INS Navigation Pipeline",
        fontsize=17,
        weight="bold",
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    ax.set_xlim(
        np.min(true_position[:, 0]) - 80,
        np.max(true_position[:, 0]) + 80,
    )
    ax.set_ylim(
        np.min(true_position[:, 1]) - 80,
        np.max(true_position[:, 1]) + 80,
    )
    ax.set_zlim(
        np.min(true_position[:, 2]) - 20,
        np.max(true_position[:, 2]) + 20,
    )

    ax.view_init(elev=25, azim=-60)
    ax.grid(True)

    sat_xy = ax.scatter(
        satellites[:, 0] / 1e5,
        satellites[:, 1] / 1e5,
        satellites[:, 2] / 1e5,
        s=35,
        color=SATELLITE_COLOR,
        marker="^",
        alpha=0.25,
        label="Satellites GNSS (échelle réduite)",
    )

    true_line, = ax.plot([], [], [], color=TRUE_COLOR, linewidth=8, label="Trajectoire vraie")
    gnss_line, = ax.plot([], [], [], color=GNSS_COLOR, alpha=0.55, label="GNSS")
    raim_line, = ax.plot([], [], [], color=RAIM_COLOR, linewidth=2, label="RAIM")
    kalman_line, = ax.plot([], [], [], color=KALMAN_COLOR, linewidth=2, label="Kalman")
    ins_line, = ax.plot([], [], [], color=INS_COLOR, alpha=0.75, label="INS")
    fusion_line, = ax.plot([], [], [], color=FUSION_COLOR, linewidth=4, label="Fusion GNSS/INS nominale")

    drone = ax.scatter([], [], [], s=80, color="black", label="Drone")

    status_text = ax.text2D(
        0.02,
        0.94,
        "",
        transform=ax.transAxes,
        fontsize=12,
        family="monospace",
        bbox=dict(facecolor="white", alpha=0.85),
    )

    frames = range(0, len(true_position), step)

    def update(frame):
        k = frame

        true_line.set_data(true_position[:k + 1, 0], true_position[:k + 1, 1])
        true_line.set_3d_properties(true_position[:k + 1, 2])

        gnss_line.set_data(gnss_position[:k + 1, 0], gnss_position[:k + 1, 1])
        gnss_line.set_3d_properties(gnss_position[:k + 1, 2])

        raim_line.set_data(raim_position[:k + 1, 0], raim_position[:k + 1, 1])
        raim_line.set_3d_properties(raim_position[:k + 1, 2])

        kalman_line.set_data(kalman_position[:k + 1, 0], kalman_position[:k + 1, 1])
        kalman_line.set_3d_properties(kalman_position[:k + 1, 2])

        ins_line.set_data(ins_position[:k + 1, 0], ins_position[:k + 1, 1])
        ins_line.set_3d_properties(ins_position[:k + 1, 2])

        fusion_line.set_data(fusion_position[:k + 1, 0], fusion_position[:k + 1, 1])
        fusion_line.set_3d_properties(fusion_position[:k + 1, 2])

        drone._offsets3d = (
            [true_position[k, 0]],
            [true_position[k, 1]],
            [true_position[k, 2]],
        )

        raim_status = "OK"
        excluded = "-"

        if raim_flags is not None and raim_flags[k]:
            raim_status = "FAULT DETECTED"

        if excluded_satellites is not None and excluded_satellites[k] is not None:
            excluded = str(excluded_satellites[k] + 1)

        status_text.set_text(
            f"t = {k:04d}\n"
            f"RAIM : {raim_status}\n"
            f"Satellite exclu : {excluded}"
        )

        if raim_status == "FAULT DETECTED":
            status_text.set_color(FAULT_COLOR)
        else:
            status_text.set_color("black")

        return (
            true_line,
            gnss_line,
            raim_line,
            kalman_line,
            ins_line,
            fusion_line,
            drone,
            status_text,
            sat_xy,
        )

    animation = FuncAnimation(
        fig,
        update,
        frames=frames,
        interval=interval,
        blit=False,
        repeat=False,
    )

    ax.legend(loc="upper right")
    plt.tight_layout()

    writer = FFMpegWriter(
        fps=30,
        bitrate=3000,
    )

    animation.save(
        "results/videos/navigation_demo.mp4",
        writer=writer,
    )

    plt.show()

    return animation