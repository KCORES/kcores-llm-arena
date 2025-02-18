import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# --- Simulation Parameters (Simplified) ---
sun_radius = 1.0  # Relative radius for visualization
earth_radius = 0.5
mars_radius = 0.3
orbit_radius_earth = 10.0
orbit_radius_mars = 15.0
earth_orbital_period = 365  # Days (arbitrary units for animation speed)
mars_orbital_period = 687  # Days (arbitrary units)
transfer_duration_earth_mars = 200  # Arbitrary units for animation
transfer_duration_mars_earth = 200  # Arbitrary units for animation
wait_on_mars_duration = 100  # Arbitrary units for animation

# Launch window timing (simplified - in reality, much more complex)
earth_mars_launch_window_phase_angle = 0  # Earth and Mars aligned (approximately)
mars_earth_launch_window_phase_angle = 180  # Earth and Mars opposite (approximately)

total_duration = (
    earth_orbital_period
    + mars_orbital_period
    + transfer_duration_earth_mars
    + transfer_duration_mars_earth
    + wait_on_mars_duration
)  # Total animation duration


# --- Initialize Plot ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_aspect("equal")  # Ensure correct proportions in 3D

# --- Planet and Sun Objects ---
# Sun (static at origin)
sun_u, sun_v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
sun_x = sun_radius * np.cos(sun_u) * np.sin(sun_v)
sun_y = sun_radius * np.sin(sun_u) * np.sin(sun_v)
sun_z = sun_radius * np.cos(sun_v)
(sun_plot,) = ax.plot_surface(sun_x, sun_y, sun_z, color="yellow", label="Sun")

# Earth
earth_u, earth_v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
earth_x_s = earth_radius * np.cos(earth_u) * np.sin(earth_v)
earth_y_s = earth_radius * np.sin(earth_u) * np.sin(earth_v)
earth_z_s = earth_radius * np.cos(earth_v)
(earth_plot,) = ax.plot_surface(
    earth_x_s, earth_y_s, earth_z_s, color="blue", label="Earth"
)
(earth_orbit_plot,) = ax.plot([], [], [], "--b", alpha=0.5)  # Earth's orbit path

# Mars
mars_u, mars_v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
mars_x_s = mars_radius * np.cos(mars_u) * np.sin(mars_v)
mars_y_s = mars_radius * np.sin(mars_u) * np.sin(mars_v)
mars_z_s = mars_radius * np.cos(mars_v)
(mars_plot,) = ax.plot_surface(mars_x_s, mars_y_s, mars_z_s, color="red", label="Mars")
(mars_orbit_plot,) = ax.plot([], [], [], "--r", alpha=0.5)  # Mars's orbit path

# Spacecraft
(spacecraft_plot,) = ax.plot(
    [0], [0], [0], marker="o", markersize=3, color="gray", label="Spacecraft"
)
(spacecraft_path_plot,) = ax.plot([], [], [], "-g", linewidth=0.5)  # Spacecraft path


# --- Trajectory Path Functions (Simplified Curves) ---
def elliptical_trajectory(start_pos, end_pos, num_points=50):
    """Generates a simplified elliptical trajectory between two points."""
    t = np.linspace(0, 1, num_points)
    control_point = (start_pos + end_pos) / 2 + np.array(
        [0, 0, 5]
    )  # Example control point for curvature
    path = (1 - t) ** 2 * start_pos + 2 * (1 - t) * t * control_point + t**2 * end_pos
    return path


# --- Animation Function ---
def animate(frame):
    ax.cla()  # Clear the axes for each frame
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Earth-Mars-Earth Mission Animation")
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-20, 20])
    ax.legend(loc="upper left")
    ax.view_init(elev=20, azim=frame * 0.5)  # Rotate view for better perspective

    # --- Planet Positions ---
    earth_angle = (frame % earth_orbital_period) / earth_orbital_period * 2 * np.pi
    mars_angle = (frame % mars_orbital_period) / mars_orbital_period * 2 * np.pi

    earth_pos = np.array(
        [
            orbit_radius_earth * np.cos(earth_angle),
            orbit_radius_earth * np.sin(earth_angle),
            0,
        ]
    )
    mars_pos = np.array(
        [
            orbit_radius_mars * np.cos(mars_angle),
            orbit_radius_mars * np.sin(mars_angle),
            0,
        ]
    )

    # Update planet positions
    earth_plot.remove()  # Need to remove and redraw for animation update to work properly with plot_surface
    mars_plot.remove()

    earth_x = earth_x_s + earth_pos[0]
    earth_y = earth_y_s + earth_pos[1]
    earth_z = earth_z_s + earth_pos[2]
    earth_plot_new = ax.plot_surface(
        earth_x, earth_y, earth_z, color="blue", label="Earth"
    )

    mars_x = mars_x_s + mars_pos[0]
    mars_y = mars_y_s + mars_pos[1]
    mars_z = mars_z_s + mars_pos[2]
    mars_plot_new = ax.plot_surface(mars_x, mars_y, mars_z, color="red", label="Mars")

    # Redraw Sun (static)
    sun_plot_new = ax.plot_surface(sun_x, sun_y, sun_z, color="yellow", label="Sun")

    # --- Planet Orbits (Paths) ---
    earth_orbit_angles = np.linspace(0, 2 * np.pi, 100)
    earth_orbit_x = orbit_radius_earth * np.cos(earth_orbit_angles)
    earth_orbit_y = orbit_radius_earth * np.sin(earth_orbit_angles)
    earth_orbit_z = np.zeros_like(earth_orbit_angles)
    (earth_orbit_plot_new,) = ax.plot(
        earth_orbit_x, earth_orbit_y, earth_orbit_z, "--b", alpha=0.5
    )

    mars_orbit_angles = np.linspace(0, 2 * np.pi, 100)
    mars_orbit_x = orbit_radius_mars * np.cos(mars_orbit_angles)
    mars_orbit_y = orbit_radius_mars * np.sin(mars_orbit_angles)
    mars_orbit_z = np.zeros_like(mars_orbit_angles)
    (mars_orbit_plot_new,) = ax.plot(
        mars_orbit_x, mars_orbit_y, mars_orbit_z, "--r", alpha=0.5
    )

    # --- Spacecraft Trajectory and Position ---
    spacecraft_pos = np.array([0, 0, 0])  # Initialize

    # Earth to Mars Transfer
    if 0 <= frame < transfer_duration_earth_mars:
        progress = frame / transfer_duration_earth_mars
        start_point = np.array(
            [
                orbit_radius_earth * np.cos(earth_mars_launch_window_phase_angle),
                orbit_radius_earth * np.sin(earth_mars_launch_window_phase_angle),
                0,
            ]
        )  # Earth's position at launch
        end_point = mars_pos  # Mars's current position (simplified approach)
        trajectory_path = elliptical_trajectory(
            start_point, end_point, transfer_duration_earth_mars
        )
        spacecraft_pos = trajectory_path[frame]
        spacecraft_path_plot.set_data(
            trajectory_path[: frame + 1, 0], trajectory_path[: frame + 1, 1]
        )
        spacecraft_path_plot.set_3d_properties(trajectory_path[: frame + 1, 2])

    # Waiting on Mars
    elif (
        transfer_duration_earth_mars
        <= frame
        < transfer_duration_earth_mars + wait_on_mars_duration
    ):
        spacecraft_pos = mars_pos  # Stay at Mars for visualization
        spacecraft_path_plot.set_data(
            trajectory_path[:, 0], trajectory_path[:, 1]
        )  # Keep displaying full path
        spacecraft_path_plot.set_3d_properties(trajectory_path[:, 2])

    # Mars to Earth Transfer (Next Launch Window - simplified)
    elif (
        transfer_duration_earth_mars + wait_on_mars_duration
        <= frame
        < transfer_duration_earth_mars
        + wait_on_mars_duration
        + transfer_duration_mars_earth
    ):
        progress = (
            frame - (transfer_duration_earth_mars + wait_on_mars_duration)
        ) / transfer_duration_mars_earth
        start_point = mars_pos  # Mars's position at Mars launch
        earth_next_launch_angle = (
            (
                (transfer_duration_earth_mars + wait_on_mars_duration)
                % earth_orbital_period
                + mars_earth_launch_window_phase_angle
            )
            / earth_orbital_period
            * 2
            * np.pi
        )  # Estimate Earth's position for return
        end_point = np.array(
            [
                orbit_radius_earth * np.cos(earth_next_launch_angle),
                orbit_radius_earth * np.sin(earth_next_launch_angle),
                0,
            ]
        )  # Earth's estimated position for return

        trajectory_path_return = elliptical_trajectory(
            start_point, end_point, transfer_duration_mars_earth
        )  # New path for return
        spacecraft_pos = trajectory_path_return[
            frame - (transfer_duration_earth_mars + wait_on_mars_duration)
        ]

        # Combine paths for plotting
        full_trajectory_path = np.concatenate((trajectory_path, trajectory_path_return))
        spacecraft_path_plot.set_data(
            full_trajectory_path[
                : (frame + 1 - (transfer_duration_earth_mars + wait_on_mars_duration))
                + transfer_duration_earth_mars,
                0,
            ],
            full_trajectory_path[
                : (frame + 1 - (transfer_duration_earth_mars + wait_on_mars_duration))
                + transfer_duration_earth_mars,
                1,
            ],
        )
        spacecraft_path_plot.set_3d_properties(
            full_trajectory_path[
                : (frame + 1 - (transfer_duration_earth_mars + wait_on_mars_duration))
                + transfer_duration_earth_mars,
                2,
            ]
        )

    # Beyond return, stay at Earth for demonstration
    elif (
        frame
        >= transfer_duration_earth_mars
        + wait_on_mars_duration
        + transfer_duration_mars_earth
    ):
        spacecraft_pos = earth_pos  # Stay at Earth after return
        full_trajectory_path = np.concatenate((trajectory_path, trajectory_path_return))
        spacecraft_path_plot.set_data(
            full_trajectory_path[:, 0], full_trajectory_path[:, 1]
        )
        spacecraft_path_plot.set_3d_properties(full_trajectory_path[:, 2])

    # Update spacecraft position
    spacecraft_plot.set_data([spacecraft_pos[0]], [spacecraft_pos[1]])
    spacecraft_plot.set_3d_properties([spacecraft_pos[2]])

    return (
        sun_plot_new,
        earth_plot_new,
        mars_plot_new,
        spacecraft_plot,
        spacecraft_path_plot,
        earth_orbit_plot_new,
        mars_orbit_plot_new,
    )


# --- Create Animation ---
ani = FuncAnimation(
    fig, animate, frames=total_duration, interval=50, blit=False
)  # Adjust interval for speed

# --- Save Animation (optional) ---
# ani.save('earth_mars_earth_mission.gif', writer='imagemagick') # Requires imagemagick to be installed
# ani.save('earth_mars_earth_mission.mp4', writer='ffmpeg') # Requires ffmpeg to be installed


plt.tight_layout()
plt.show()
