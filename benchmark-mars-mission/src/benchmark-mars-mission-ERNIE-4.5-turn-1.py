import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants for simplicity
AU = 1.496e+11  # Astronomical unit in meters
earth_orbit_radius = 1 * AU
mars_orbit_radius = 1.524 * AU

# Time settings
total_time = 10  # years (arbitrary for illustration)
frames = 1000
dt = total_time / frames

# Generate time array
time = np.linspace(0, total_time, frames)

# Simulate positions (simplified circular orbits)
def get_position(radius, time, period):
    angle = 2 * np.pi * time / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = np.zeros_like(x)  # Assume orbits are in the XY plane
    return x, y, z

# Orbital periods in years converted to the same time units
earth_period = 1  # 1 year
mars_period = 1.88  # Mars orbital period in years

# Get Earth and Mars positions
earth_x, earth_y, earth_z = get_position(earth_orbit_radius, time, earth_period)
mars_x, mars_y, mars_z = get_position(mars_orbit_radius, time, mars_period)

# Simulate a simple transfer trajectory (straight line for illustration)
# This is a naive approximation; real transfers use Hohmann or other orbits
spacecraft_x = np.concatenate([
    np.linspace(earth_x[0], mars_x[250], 250),  # To Mars
    np.linspace(mars_x[250], earth_x[500], 250),  # To Earth (next window)
    np.linspace(earth_x[500], earth_x[-1], frames - 500)  # Return to Earth vicinity
])

spacecraft_y = np.concatenate([
    np.linspace(earth_y[0], mars_y[250], 250),
    np.linspace(mars_y[250], earth_y[500], 250),
    np.linspace(earth_y[500], earth_y[-1], frames - 500)
])

spacecraft_z = np.zeros_like(spacecraft_x)  # Keep it simple in 2D plane for now

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Setting limits and labels
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.1 * AU, 0.1 * AU])  # Focus on the XY plane
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')

# Plot initial positions
earth_line, = ax.plot([], [], [], 'bo', label='Earth')
mars_line, = ax.plot([], [], [], 'ro', label='Mars')
spacecraft_line, = ax.plot([], [], [], 'go', label='Spacecraft')

# Add orbit paths for visualization
ax.plot(earth_x, earth_y, earth_z, 'b--', alpha=0.5)
ax.plot(mars_x, mars_y, mars_z, 'r--', alpha=0.5)

ax.legend()

# Animation function
def update(frame):
    earth_line.set_data(earth_x[frame], earth_y[frame])
    earth_line.set_3d_properties(earth_z[frame])
    
    mars_line.set_data(mars_x[frame], mars_y[frame])
    mars_line.set_3d_properties(mars_z[frame])
    
    spacecraft_line.set_data(spacecraft_x[frame], spacecraft_y[frame])
    spacecraft_line.set_3d_properties(spacecraft_z[frame])
    
    return earth_line, mars_line, spacecraft_line

# Create animation
ani = FuncAnimation(fig, update, frames=frames, interval=50, blit=False)

plt.show()