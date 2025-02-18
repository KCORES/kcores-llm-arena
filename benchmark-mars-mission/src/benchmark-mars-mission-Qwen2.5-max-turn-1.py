import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
earth_orbit_radius = 1.0  # AU (Astronomical Unit)
mars_orbit_radius = 1.52  # AU
orbit_speed_earth = 2 * np.pi / 365  # Radians per day
orbit_speed_mars = 2 * np.pi / 687  # Radians per day
launch_window_days = 780  # Approximate synodic period of Earth and Mars in days

# Time array for animation (in days)
time_steps = np.linspace(0, launch_window_days, 500)

# Generate Earth's and Mars's positions over time
earth_x = earth_orbit_radius * np.cos(orbit_speed_earth * time_steps)
earth_y = earth_orbit_radius * np.sin(orbit_speed_earth * time_steps)
mars_x = mars_orbit_radius * np.cos(orbit_speed_mars * time_steps)
mars_y = mars_orbit_radius * np.sin(orbit_speed_mars * time_steps)

# Interpolate spacecraft trajectory
# Assume spacecraft travels along a straight line between Earth and Mars
spacecraft_x = np.linspace(earth_x[0], mars_x[len(mars_x) // 2], len(time_steps) // 2)
spacecraft_y = np.linspace(earth_y[0], mars_y[len(mars_y) // 2], len(time_steps) // 2)
# Return trajectory
return_x = np.linspace(mars_x[len(mars_x) // 2], earth_x[-1], len(time_steps) // 2)
return_y = np.linspace(mars_y[len(mars_y) // 2], earth_y[-1], len(time_steps) // 2)
# Combine trajectories
spacecraft_x = np.concatenate([spacecraft_x, return_x])
spacecraft_y = np.concatenate([spacecraft_y, return_y])

# Set up the 3D plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot Earth's orbit
ax.plot(earth_x, earth_y, zs=0, zdir="z", label="Earth's Orbit", color="blue")
# Plot Mars's orbit
ax.plot(mars_x, mars_y, zs=0, zdir="z", label="Mars's Orbit", color="red")

# Initialize spacecraft trajectory
(spacecraft_line,) = ax.plot(
    [], [], [], label="Spacecraft Trajectory", color="green", lw=2
)
(spacecraft_dot,) = ax.plot([], [], [], "o", color="black", markersize=5)

# Set limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.1, 0.1)  # Flattened view
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.legend()


# Animation update function
def update(frame):
    # Update spacecraft trajectory
    spacecraft_line.set_data(spacecraft_x[:frame], spacecraft_y[:frame])
    spacecraft_line.set_3d_properties(np.zeros(frame))  # Z-coordinate is 0
    # Update spacecraft position
    spacecraft_dot.set_data(spacecraft_x[frame], spacecraft_y[frame])
    spacecraft_dot.set_3d_properties(0)  # Z-coordinate is 0
    return spacecraft_line, spacecraft_dot


# Create animation
ani = FuncAnimation(fig, update, frames=len(spacecraft_x), interval=50, blit=True)

# Show the animation
plt.show()
