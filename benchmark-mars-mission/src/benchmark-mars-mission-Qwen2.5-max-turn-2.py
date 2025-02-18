import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
earth_orbit_radius = 1  # AU (Astronomical Unit)
mars_orbit_radius = 1.524  # AU
orbit_speed_earth = 2 * np.pi / 365  # Radians per day
orbit_speed_mars = 2 * np.pi / 687  # Radians per day
launch_window_days = 780  # Approximate synodic period of Earth and Mars in days

# Time array for animation (in days)
time_steps = np.linspace(0, launch_window_days, 500)

# Generate Earth and Mars positions over time
earth_x = earth_orbit_radius * np.cos(orbit_speed_earth * time_steps)
earth_y = earth_orbit_radius * np.sin(orbit_speed_earth * time_steps)
mars_x = mars_orbit_radius * np.cos(orbit_speed_mars * time_steps)
mars_y = mars_orbit_radius * np.sin(orbit_speed_mars * time_steps)

# Spacecraft trajectory: Interpolate between Earth and Mars, then back to Earth
spacecraft_x = np.concatenate(
    [
        np.linspace(earth_x[0], mars_x[len(mars_x) // 2], len(time_steps) // 2),
        np.linspace(mars_x[len(mars_x) // 2], earth_x[-1], len(time_steps) // 2),
    ]
)
spacecraft_y = np.concatenate(
    [
        np.linspace(earth_y[0], mars_y[len(mars_y) // 2], len(time_steps) // 2),
        np.linspace(mars_y[len(mars_y) // 2], earth_y[-1], len(time_steps) // 2),
    ]
)

# Set up the 3D plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot the orbits of Earth and Mars
ax.plot(earth_x, earth_y, zs=0, zdir="z", label="Earth Orbit", color="blue")
ax.plot(mars_x, mars_y, zs=0, zdir="z", label="Mars Orbit", color="red")

# Initialize spacecraft and planets
(earth_point,) = ax.plot([], [], [], "o", color="blue", markersize=10, label="Earth")
(mars_point,) = ax.plot([], [], [], "o", color="red", markersize=10, label="Mars")
(spacecraft_point,) = ax.plot(
    [], [], [], "o", color="green", markersize=5, label="Spacecraft"
)
(trajectory_line,) = ax.plot(
    [], [], [], "-", color="green", alpha=0.5, label="Trajectory"
)

# Set plot limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.5, 0.5)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.legend()


# Animation update function
def update(frame):
    # Update Earth and Mars positions
    earth_point.set_data([earth_x[frame]], [earth_y[frame]])
    earth_point.set_3d_properties(0)
    mars_point.set_data([mars_x[frame]], [mars_y[frame]])
    mars_point.set_3d_properties(0)

    # Update spacecraft position
    if frame < len(spacecraft_x):
        spacecraft_point.set_data([spacecraft_x[frame]], [spacecraft_y[frame]])
        spacecraft_point.set_3d_properties(0)
        trajectory_line.set_data(spacecraft_x[:frame], spacecraft_y[:frame])
        trajectory_line.set_3d_properties(np.zeros(frame))

    return earth_point, mars_point, spacecraft_point, trajectory_line


# Create the animation
ani = FuncAnimation(fig, update, frames=len(time_steps), interval=50, blit=True)

# Show the animation
plt.show()
