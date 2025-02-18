import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Set up figure and 3D axes
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Parameters
t = np.linspace(0, 4 * np.pi, 200)  # Time parameter for animation
earth_orbit_radius = 1.0  # AU (Astronomical Units)
mars_orbit_radius = 1.524  # AU
rocket_speed = 0.05  # Arbitrary units for animation

# Planetary orbits
earth_x = earth_orbit_radius * np.cos(t)
earth_y = earth_orbit_radius * np.sin(t)
mars_x = mars_orbit_radius * np.cos(t * 0.531)  # Mars orbital period ~1.88 Earth years
mars_y = mars_orbit_radius * np.sin(t * 0.531)


# Simplified transfer orbit (Hohmann-like trajectory)
def transfer_trajectory(start_radius, end_radius, t):
    a = (start_radius + end_radius) / 2  # Semi-major axis
    x = a * np.cos(t)
    y = a * np.sin(t) * np.sqrt(1 - (start_radius / a) ** 2)  # Elliptical path
    return x, y, np.zeros_like(x)


# Animation setup
(rocket,) = ax.plot([], [], [], "r-", lw=2, label="Rocket")
(earth,) = ax.plot([], [], [], "bo", label="Earth")
(mars,) = ax.plot([], [], [], "ro", label="Mars")
(path,) = ax.plot([], [], [], "g--", alpha=0.5)


def init():
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-1, 1)
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Earth-Mars-Earth Trajectory")
    ax.legend()
    return rocket, earth, mars, path


def update(frame):
    # Reset plots
    rocket.set_data([], [])
    rocket.set_3d_properties([])
    earth.set_data([], [])
    earth.set_3d_properties([])
    mars.set_data([], [])
    mars.set_3d_properties([])
    path.set_data([], [])
    path.set_3d_properties([])

    t_current = frame * 0.1

    # Earth and Mars positions
    earth_pos_x = earth_orbit_radius * np.cos(t_current)
    earth_pos_y = earth_orbit_radius * np.sin(t_current)
    mars_pos_x = mars_orbit_radius * np.cos(t_current * 0.531)
    mars_pos_y = mars_orbit_radius * np.sin(t_current * 0.531)

    earth.set_data([earth_pos_x], [earth_pos_y])
    earth.set_3d_properties([0])
    mars.set_data([mars_pos_x], [mars_pos_y])
    mars.set_3d_properties([0])

    # Trip phases
    total_frames = 200
    trip1_end = total_frames // 3  # Earth to Mars
    stay_end = 2 * total_frames // 3  # Stay on Mars
    trip2_end = total_frames  # Mars back to Earth

    if frame < trip1_end:  # Earth to Mars
        t_trans = np.linspace(0, np.pi, trip1_end)[:frame]
        x, y, z = transfer_trajectory(earth_orbit_radius, mars_orbit_radius, t_trans)
        rocket.set_data([x[-1]], [y[-1]])
        rocket.set_3d_properties([z[-1]])
        path.set_data(x, y)
        path.set_3d_properties(z)

    elif frame < stay_end:  # On Mars
        rocket.set_data([mars_pos_x], [mars_pos_y])
        rocket.set_3d_properties([0])
        # Show complete outbound path
        t_trans = np.linspace(0, np.pi, trip1_end)
        x, y, z = transfer_trajectory(earth_orbit_radius, mars_orbit_radius, t_trans)
        path.set_data(x, y)
        path.set_3d_properties(z)

    else:  # Mars to Earth
        t_trans = np.linspace(np.pi, 2 * np.pi, trip2_end - stay_end)[
            : frame - stay_end
        ]
        x, y, z = transfer_trajectory(mars_orbit_radius, earth_orbit_radius, t_trans)
        rocket.set_data([x[-1]], [y[-1]])
        rocket.set_3d_properties([z[-1]])
        # Show both paths
        t_out = np.linspace(0, np.pi, trip1_end)
        x_out, y_out, z_out = transfer_trajectory(
            earth_orbit_radius, mars_orbit_radius, t_out
        )
        x_ret, y_ret, z_ret = transfer_trajectory(
            mars_orbit_radius, earth_orbit_radius, t_trans
        )
        path.set_data(np.concatenate([x_out, x_ret]), np.concatenate([y_out, y_ret]))
        path.set_3d_properties(np.concatenate([z_out, z_ret]))

    return rocket, earth, mars, path


# Create animation
ani = animation.FuncAnimation(
    fig, update, frames=200, init_func=init, interval=50, blit=True
)

plt.show()
