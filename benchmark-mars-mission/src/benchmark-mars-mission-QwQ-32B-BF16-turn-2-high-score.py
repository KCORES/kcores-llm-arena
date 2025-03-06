import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.0  # Astronomical Unit
days_in_year = 365.25  # Days per Earth year
earth_radius = 1.0  # Earth's orbital radius (AU)
mars_radius = 1.5  # Mars' orbital radius (AU)

# Angular velocities (radians per day)
omega_earth = 2 * np.pi / days_in_year
omega_mars = 2 * np.pi / 687  # Mars orbital period ~687 days

# Hohmann transfer parameters
a_transfer = (earth_radius + mars_radius) / 2  # Semi-major axis of transfer orbit
T_transfer_days = np.sqrt(a_transfer**3) * days_in_year  # Total period in days
T = T_transfer_days / 2  # One-way transfer time (days)
total_time = 2 * T  # Total time for round trip

# Animation parameters
dt = total_time / 500  # Time step between frames
time_steps = np.arange(0, total_time + dt, dt)

# Setup figure and axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.5, 0.5)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars Spacecraft Journey")

# Plot Sun
ax.plot([0], [0], [0], "yo", markersize=12, label="Sun")

# Orbit lines
earth_orbit = ax.plot(
    earth_radius * np.cos(np.linspace(0, 2 * np.pi, 100)),
    earth_radius * np.sin(np.linspace(0, 2 * np.pi, 100)),
    zs=0,
    color="blue",
    alpha=0.5,
)[0]
mars_orbit = ax.plot(
    mars_radius * np.cos(np.linspace(0, 2 * np.pi, 100)),
    mars_radius * np.sin(np.linspace(0, 2 * np.pi, 100)),
    zs=0,
    color="red",
    alpha=0.5,
)[0]

# Earth and Mars positions
(earth_pos,) = ax.plot([], [], [], "bo", markersize=6, label="Earth")
(mars_pos,) = ax.plot([], [], [], "ro", markersize=4, label="Mars")

# Spacecraft path and current position
(spacecraft_path,) = ax.plot(
    [], [], [], color="green", alpha=0.7, lw=1.5, label="Trajectory"
)
(spacecraft_point,) = ax.plot([], [], [], "go", markersize=4, label="Spacecraft")

ax.legend(loc="upper right")


# Precompute spacecraft path for smoother animation
def get_spacecraft_position(time):
    if time <= T:
        earth_start = np.array([earth_radius, 0, 0])
        mars_end_angle = omega_mars * T
        mars_end = np.array(
            [
                mars_radius * np.cos(mars_end_angle),
                mars_radius * np.sin(mars_end_angle),
                0.0,
            ]
        )
        pos = earth_start + (mars_end - earth_start) * (time / T)
    else:
        mars_start_angle = omega_mars * T
        mars_start = np.array(
            [
                mars_radius * np.cos(mars_start_angle),
                mars_radius * np.sin(mars_start_angle),
                0.0,
            ]
        )
        earth_end_angle = omega_earth * (2 * T)
        earth_end = np.array(
            [
                earth_radius * np.cos(earth_end_angle),
                earth_radius * np.sin(earth_end_angle),
                0.0,
            ]
        )
        pos = mars_start + (earth_end - mars_start) * ((time - T) / T)
    return pos


path_points = [get_spacecraft_position(t) for t in time_steps]
path_x = [p[0] for p in path_points]
path_y = [p[1] for p in path_points]
path_z = [p[2] for p in path_points]


def init():
    spacecraft_path.set_data([], [])
    spacecraft_path.set_3d_properties([])
    spacecraft_point.set_data([], [])
    spacecraft_point.set_3d_properties([])
    earth_pos.set_data([], [])
    earth_pos.set_3d_properties([])
    mars_pos.set_data([], [])
    mars_pos.set_3d_properties([])
    return (spacecraft_path, spacecraft_point, earth_pos, mars_pos)


def animate(frame):
    t = frame * dt

    # Update Earth position
    earth_angle = omega_earth * t
    earth_x = earth_radius * np.cos(earth_angle)
    earth_y = earth_radius * np.sin(earth_angle)
    earth_pos.set_data([earth_x], [earth_y])
    earth_pos.set_3d_properties([0])

    # Update Mars position
    mars_angle = omega_mars * t
    mars_x = mars_radius * np.cos(mars_angle)
    mars_y = mars_radius * np.sin(mars_angle)
    mars_pos.set_data([mars_x], [mars_y])
    mars_pos.set_3d_properties([0])

    # Update spacecraft path and current position
    idx = int(t / dt)
    spacecraft_path.set_data(path_x[:idx], path_y[:idx])
    spacecraft_path.set_3d_properties(path_z[:idx])
    sc_pos = get_spacecraft_position(t)
    spacecraft_point.set_data([sc_pos[0]], [sc_pos[1]])
    spacecraft_point.set_3d_properties([sc_pos[2]])

    return (spacecraft_path, spacecraft_point, earth_pos, mars_pos)


ani = FuncAnimation(
    fig, animate, frames=int(total_time / dt), init_func=init, blit=True, interval=30
)
plt.tight_layout()
plt.show()
