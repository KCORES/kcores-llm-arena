import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e+11  # Astronomical unit in meters
days_to_sec = 86400  # Conversion from days to seconds

# Simplified orbits (circular for visualization)
earth_orbit_radius = 1 * AU
mars_orbit_radius = 1.524 * AU

# Time settings
total_time = 730  # days (approximately 2 years for a Hohmann transfer)
fps = 30
total_frames = int(total_time * days_to_sec / (1/fps))

# Spacecraft trajectory (simplified straight-line Hohmann transfer for illustration)
def spacecraft_trajectory(t):
    # Simulate a transfer orbit using parametric equations
    # This is a very simplified model; real transfers are elliptical
    if t < total_time / 4:
        # Leaving Earth
        x = earth_orbit_radius * np.cos(2 * np.pi * t / (total_time / 4))
        y = earth_orbit_radius * np.sin(2 * np.pi * t / (total_time / 4))
        z = 0
        x += (mars_orbit_radius - earth_orbit_radius) * (t / (total_time / 4))  # Simplistic transition
    elif t < 3 * total_time / 4:
        # At Mars (hovering at Mars' distance for simplicity)
        x = mars_orbit_radius
        y = mars_orbit_radius * np.sin(2 * np.pi * (t - total_time / 4) / (total_time / 2))
        z = 0
    else:
        # Returning to Earth
        progress = (t - 3 * total_time / 4) / (total_time / 4)
        x = mars_orbit_radius - (mars_orbit_radius - earth_orbit_radius) * progress
        y = earth_orbit_radius * np.sin(2 * np.pi * progress)  # Simplified return path
        z = 0
    return x, y, z

# Set up the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot Earth and Mars orbits
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit_x = earth_orbit_radius * np.cos(theta)
earth_orbit_y = earth_orbit_radius * np.sin(theta)
mars_orbit_x = mars_orbit_radius * np.cos(theta)
mars_orbit_y = mars_orbit_radius * np.sin(theta)

ax.plot(earth_orbit_x, earth_orbit_y, 0, 'blue', label='Earth Orbit')
ax.plot(mars_orbit_x, mars_orbit_y, 0, 'red', label='Mars Orbit')

# Initialize spacecraft position
spacecraft_pos, = ax.plot([], [], [], 'o', label='Spacecraft')

# Animation function
def update(frame):
    t = frame / fps  # Convert frame number to time in seconds
    x, y, z = spacecraft_trajectory(t)
    spacecraft_pos.set_data(x, y)
    spacecraft_pos.set_3d_properties(z)
    return spacecraft_pos,

# Create animation
ani = FuncAnimation(fig, update, frames=total_frames, blit=True, interval=1000/fps)

# Labels and legend
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.legend()

plt.show()