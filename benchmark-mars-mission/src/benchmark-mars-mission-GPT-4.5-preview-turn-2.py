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
launch_window = 780  # days (约为2年2个月)

# Time settings
total_days = 2 * launch_window
frames = 500
time = np.linspace(0, total_days, frames)


# Orbit calculations
def orbit(radius, period, t):
    angle = 2 * np.pi * t / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros_like(x)
    return x, y, z


# Earth and Mars positions
earth_x, earth_y, earth_z = orbit(earth_orbit_radius, earth_period, time)
mars_x, mars_y, mars_z = orbit(mars_orbit_radius, mars_period, time)


# Spacecraft trajectory (simplified elliptical transfer orbit)
def spacecraft_trajectory(t):
    # First half: Earth to Mars
    if t <= launch_window / 2:
        ratio = t / (launch_window / 2)
        x = (1 - ratio) * earth_x[0] + ratio * mars_x[int(frames / 2)]
        y = (1 - ratio) * earth_y[0] + ratio * mars_y[int(frames / 2)]
    # Second half: Mars to Earth
    else:
        ratio = (t - launch_window / 2) / (launch_window / 2)
        x = (1 - ratio) * mars_x[int(frames / 2)] + ratio * earth_x[-1]
        y = (1 - ratio) * mars_y[int(frames / 2)] + ratio * earth_y[-1]
    z = 0
    return x, y, z


# Set up plot
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2 * AU, 2 * AU)
ax.set_ylim(-2 * AU, 2 * AU)
ax.set_zlim(-0.5 * AU, 0.5 * AU)
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Mission Animation")

# Plot orbits
ax.plot(earth_x, earth_y, earth_z, color="blue", linestyle="--", label="Earth Orbit")
ax.plot(mars_x, mars_y, mars_z, color="red", linestyle="--", label="Mars Orbit")

# Initialize spacecraft and planets
(earth_dot,) = ax.plot([], [], [], "bo", markersize=10, label="Earth")
(mars_dot,) = ax.plot([], [], [], "ro", markersize=8, label="Mars")
(craft_dot,) = ax.plot([], [], [], "go", markersize=6, label="Spacecraft")
(trajectory_line,) = ax.plot([], [], [], "g-", linewidth=1, alpha=0.7)


# Animation function
def animate(i):
    earth_dot.set_data(earth_x[i], earth_y[i])
    earth_dot.set_3d_properties(earth_z[i])

    mars_dot.set_data(mars_x[i], mars_y[i])
    mars_dot.set_3d_properties(mars_z[i])

    craft_x, craft_y, craft_z = spacecraft_trajectory(time[i])
    craft_dot.set_data(craft_x, craft_y)
    craft_dot.set_3d_properties(craft_z)

    # Update trajectory line
    traj_x = []
    traj_y = []
    traj_z = []
    for j in range(i + 1):
        x, y, z = spacecraft_trajectory(time[j])
        traj_x.append(x)
        traj_y.append(y)
        traj_z.append(z)
    trajectory_line.set_data(traj_x, traj_y)
    trajectory_line.set_3d_properties(traj_z)

    return earth_dot, mars_dot, craft_dot, trajectory_line


# Create animation
ani = FuncAnimation(fig, animate, frames=frames, interval=50, blit=True)

ax.legend()
plt.show()
