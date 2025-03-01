import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.496e8  # km
earth_orbit_radius = 1 * AU
mars_orbit_radius = 1.524 * AU
earth_period = 365.25  # days
mars_period = 687  # days
launch_window = 780  # days (约2年2个月)

# Time settings
total_days = 2 * launch_window
frames = 500
t = np.linspace(0, total_days, frames)


# Orbit calculations
def orbit(radius, period, t):
    angle = 2 * np.pi * t / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros_like(x)
    return x, y, z


# Earth and Mars orbits
earth_x, earth_y, earth_z = orbit(earth_orbit_radius, earth_period, t)
mars_x, mars_y, mars_z = orbit(mars_orbit_radius, mars_period, t)


# Spacecraft trajectory (simplified elliptical transfer orbit)
def spacecraft_trajectory(t):
    traj_x = np.zeros_like(t)
    traj_y = np.zeros_like(t)
    traj_z = np.zeros_like(t)
    for i, day in enumerate(t):
        if day <= launch_window / 2:
            # Earth to Mars
            frac = day / (launch_window / 2)
            traj_x[i] = (1 - frac) * earth_x[i] + frac * mars_x[i]
            traj_y[i] = (1 - frac) * earth_y[i] + frac * mars_y[i]
        elif day <= launch_window:
            # Stay on Mars
            traj_x[i] = mars_x[i]
            traj_y[i] = mars_y[i]
        elif day <= 1.5 * launch_window:
            # Mars to Earth
            frac = (day - launch_window) / (launch_window / 2)
            traj_x[i] = (1 - frac) * mars_x[i] + frac * earth_x[i]
            traj_y[i] = (1 - frac) * mars_y[i] + frac * earth_y[i]
        else:
            # Stay on Earth
            traj_x[i] = earth_x[i]
            traj_y[i] = earth_y[i]
    return traj_x, traj_y, traj_z


sc_x, sc_y, sc_z = spacecraft_trajectory(t)

# Plot setup
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot orbits
ax.plot(earth_x, earth_y, earth_z, "b--", label="Earth Orbit")
ax.plot(mars_x, mars_y, mars_z, "r--", label="Mars Orbit")

# Plot Sun
ax.scatter(0, 0, 0, color="yellow", s=300, label="Sun")

# Initialize spacecraft and planets
(earth_dot,) = ax.plot([], [], [], "bo", markersize=10, label="Earth")
(mars_dot,) = ax.plot([], [], [], "ro", markersize=8, label="Mars")
(sc_dot,) = ax.plot([], [], [], "go", markersize=6, label="Spacecraft")
(sc_traj,) = ax.plot([], [], [], "g-", linewidth=1)

# Set plot limits
limit = mars_orbit_radius * 1.2
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-limit / 2, limit / 2)

ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Mission Animation")
ax.legend()


# Animation function
def update(frame):
    earth_dot.set_data(earth_x[frame], earth_y[frame])
    earth_dot.set_3d_properties(earth_z[frame])

    mars_dot.set_data(mars_x[frame], mars_y[frame])
    mars_dot.set_3d_properties(mars_z[frame])

    sc_dot.set_data(sc_x[frame], sc_y[frame])
    sc_dot.set_3d_properties(sc_z[frame])

    sc_traj.set_data(sc_x[:frame], sc_y[:frame])
    sc_traj.set_3d_properties(sc_z[:frame])

    return earth_dot, mars_dot, sc_dot, sc_traj


ani = FuncAnimation(fig, update, frames=frames, interval=50, blit=True)

plt.show()
