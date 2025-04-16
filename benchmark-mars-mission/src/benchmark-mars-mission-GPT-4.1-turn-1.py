import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

# Parameters
earth_radius = 1  # arbitrary units
mars_radius = 1.5  # arbitrary units
earth_orbit_radius = 8
mars_orbit_radius = 14
n_frames_to_mars = 150
n_frames_on_mars = 40
n_frames_to_earth = 150
total_frames = n_frames_to_mars + n_frames_on_mars + n_frames_to_earth


# Create planetary positions as functions of time (simple circles)
def get_planet_pos(theta, orbit_radius):
    return (
        orbit_radius * np.cos(theta),
        orbit_radius * np.sin(theta),
        0 * theta,  # All on XY plane
    )


# Calculate transfer arc
def get_transfer_arc(start, end, t):
    # Simple elliptical transfer between start and end positions
    # t from 0 to 1
    # Use half ellipse above plane for visual
    mid = (start + end) / 2
    r = np.linalg.norm(end - start) / 2
    center = (start + end) / 2
    dir_vec = (end - start) / np.linalg.norm(end - start)
    orbital_normal = np.array([0, 0, 1])
    ortho_vec = np.cross(dir_vec, orbital_normal)
    ortho_vec = ortho_vec / np.linalg.norm(ortho_vec)
    pos = (
        center
        + r * np.cos(np.pi * t) * dir_vec
        + (r / 1.5) * np.sin(np.pi * t) * ortho_vec  # Raise above plane
    )
    return pos


# Precompute planetary orbits for overlapping motion
earth_theta = np.linspace(0, 2 * np.pi, total_frames)
mars_theta = np.linspace(
    np.pi / 3, 2 * np.pi + np.pi / 3, total_frames
)  # little ahead start

earth_pos = np.array(get_planet_pos(earth_theta, earth_orbit_radius)).T
mars_pos = np.array(get_planet_pos(mars_theta, mars_orbit_radius)).T

# Precompute spacecraft positions
sc_pos = np.zeros((total_frames, 3))
# Launch to Mars
sc_pos[:n_frames_to_mars] = [
    get_transfer_arc(earth_pos[i], mars_pos[i], i / (n_frames_to_mars - 1))
    for i in range(n_frames_to_mars)
]
# Wait on Mars
sc_pos[n_frames_to_mars : n_frames_to_mars + n_frames_on_mars] = mars_pos[
    n_frames_to_mars : n_frames_to_mars + n_frames_on_mars
]
# Return transfer
for j, i in enumerate(range(n_frames_to_mars + n_frames_on_mars, total_frames)):
    t = j / (n_frames_to_earth - 1)
    sc_pos[i] = get_transfer_arc(mars_pos[i], earth_pos[i], t)

# Set up 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
ax.set_aspect("auto")

# Plot Earth and Mars orbits
(orbit1,) = ax.plot([], [], [], "b--", lw=0.6)
(orbit2,) = ax.plot([], [], [], "r--", lw=0.6)

# Planets & spaceship (as points)
earth_surf = ax.plot_surface(
    *[
        earth_radius
        * np.outer(
            np.cos(np.linspace(0, 2 * np.pi, 30)), np.sin(np.linspace(0, np.pi, 15))
        )
        + earth_orbit_radius,  # x
        earth_radius
        * np.outer(
            np.sin(np.linspace(0, 2 * np.pi, 30)), np.sin(np.linspace(0, np.pi, 15))
        ),
        earth_radius * np.outer(np.ones(30), np.cos(np.linspace(0, np.pi, 15))),
    ],
    color="b",
    alpha=0.4
)
mars_surf = ax.plot_surface(
    *[
        mars_radius
        * np.outer(
            np.cos(np.linspace(0, 2 * np.pi, 30)), np.sin(np.linspace(0, np.pi, 15))
        )
        + mars_orbit_radius,  # x
        mars_radius
        * np.outer(
            np.sin(np.linspace(0, 2 * np.pi, 30)), np.sin(np.linspace(0, np.pi, 15))
        ),
        mars_radius * np.outer(np.ones(30), np.cos(np.linspace(0, np.pi, 15))),
    ],
    color="r",
    alpha=0.4
)
(planet1,) = ax.plot([], [], [], "bo", markersize=10, label="Earth")
(planet2,) = ax.plot([], [], [], "ro", markersize=10, label="Mars")
(sc_marker,) = ax.plot([], [], [], "ko", markersize=7, label="Spacecraft")

# Set limits
lim = mars_orbit_radius + 3
ax.set_xlim([-lim, lim])
ax.set_ylim([-lim, lim])
ax.set_zlim([-lim / 2, lim / 2])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Earth-Mars-Earth Launch Animation")
ax.legend()


def animate(i):
    # Move planet markers
    planet1.set_data(earth_pos[i, 0], earth_pos[i, 1])
    planet1.set_3d_properties(earth_pos[i, 2])

    planet2.set_data(mars_pos[i, 0], mars_pos[i, 1])
    planet2.set_3d_properties(mars_pos[i, 2])

    # Move spacecraft
    sc_marker.set_data(sc_pos[i, 0], sc_pos[i, 1])
    sc_marker.set_3d_properties(sc_pos[i, 2])

    # Orbits (static)
    t = np.linspace(0, 2 * np.pi, 100)
    x1, y1, z1 = get_planet_pos(t, earth_orbit_radius)
    orbit1.set_data(x1, y1)
    orbit1.set_3d_properties(z1)
    x2, y2, z2 = get_planet_pos(t, mars_orbit_radius)
    orbit2.set_data(x2, y2)
    orbit2.set_3d_properties(z2)
    return planet1, planet2, sc_marker, orbit1, orbit2


# Remove initial surfs for animation (they plot as "initial")
ax.collections.clear()

ani = animation.FuncAnimation(fig, animate, frames=total_frames, interval=30, blit=True)

plt.show()
