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


# Orbit positions
def orbit(radius, period, t):
    angle = 2 * np.pi * t / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros_like(x)
    return x, y, z


earth_x, earth_y, earth_z = orbit(earth_orbit_radius, earth_period, t)
mars_x, mars_y, mars_z = orbit(mars_orbit_radius, mars_period, t)


# Spacecraft trajectory (simplified elliptical transfer orbit)
def spacecraft_trajectory(t):
    traj_x, traj_y, traj_z = [], [], []
    for day in t:
        if day <= launch_window / 2:
            # Earth to Mars
            frac = day / (launch_window / 2)
            x = (1 - frac) * earth_orbit_radius * np.cos(
                2 * np.pi * day / earth_period
            ) + frac * mars_orbit_radius * np.cos(2 * np.pi * day / mars_period)
            y = (1 - frac) * earth_orbit_radius * np.sin(
                2 * np.pi * day / earth_period
            ) + frac * mars_orbit_radius * np.sin(2 * np.pi * day / mars_period)
        elif day <= launch_window:
            # Stay on Mars orbit
            angle = 2 * np.pi * day / mars_period
            x = mars_orbit_radius * np.cos(angle)
            y = mars_orbit_radius * np.sin(angle)
        elif day <= 1.5 * launch_window:
            # Mars to Earth
            frac = (day - launch_window) / (launch_window / 2)
            x = (1 - frac) * mars_orbit_radius * np.cos(
                2 * np.pi * day / mars_period
            ) + frac * earth_orbit_radius * np.cos(2 * np.pi * day / earth_period)
            y = (1 - frac) * mars_orbit_radius * np.sin(
                2 * np.pi * day / mars_period
            ) + frac * earth_orbit_radius * np.sin(2 * np.pi * day / earth_period)
        else:
            # Stay on Earth orbit
            angle = 2 * np.pi * day / earth_period
            x = earth_orbit_radius * np.cos(angle)
            y = earth_orbit_radius * np.sin(angle)
        traj_x.append(x)
        traj_y.append(y)
        traj_z.append(0)
    return traj_x, traj_y, traj_z


sc_x, sc_y, sc_z = spacecraft_trajectory(t)

# Plot setup
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot orbits
ax.plot(earth_x, earth_y, earth_z, color="blue", linestyle="--", label="Earth Orbit")
ax.plot(mars_x, mars_y, mars_z, color="red", linestyle="--", label="Mars Orbit")

# Plot Sun
ax.scatter(0, 0, 0, color="yellow", s=300, label="Sun")

# Spacecraft and planets
(earth_dot,) = ax.plot([], [], [], "bo", markersize=8, label="Earth")
(mars_dot,) = ax.plot([], [], [], "ro", markersize=6, label="Mars")
(sc_dot,) = ax.plot([], [], [], "go", markersize=4, label="Spacecraft")
(sc_traj,) = ax.plot([], [], [], "g-", linewidth=1, alpha=0.5)


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


# Axes settings
ax.set_xlim(-2 * AU, 2 * AU)
ax.set_ylim(-2 * AU, 2 * AU)
ax.set_zlim(-0.5 * AU, 0.5 * AU)
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Mission Animation")
ax.legend()

ani = FuncAnimation(fig, update, frames=frames, interval=50, blit=True)

plt.show()
