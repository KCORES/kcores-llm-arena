import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.496e+11  # Astronomical unit in meters
days_in_year = 365.25

# Orbital parameters (simplified)
earth_orbit_radius = 1 * AU
mars_orbit_radius = 1.524 * AU

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Time settings
total_days = 10 * days_in_year  # Arbitrary long period for animation purposes
dt = 1  # Time step in days
times = np.arange(0, total_days, dt)

# Generate Earth and Mars positions
theta_earth = 2 * np.pi * times / days_in_year
theta_mars = 2 * np.pi * times / (1.88 * days_in_year)  # Mars year is about 1.88 Earth years

earth_positions = np.array([earth_orbit_radius * np.cos(theta_earth),
                            earth_orbit_radius * np.sin(theta_earth),
                            np.zeros_like(theta_earth)]).T

mars_positions = np.array([mars_orbit_radius * np.cos(theta_mars),
                           mars_orbit_radius * np.sin(theta_mars),
                           np.zeros_like(theta_mars)]).T

# Simulate a simple spacecraft trajectory (straight line approximation for illustration)
# This is a placeholder for a real trajectory calculation
spacecraft_positions = []
launch_index = 0  # Launch from Earth
intercept_index = int(total_days / 3)  # Arbitrary intercept with Mars
return_index = 2 * intercept_index  # Return to Earth at next window

for i, t in enumerate(times):
    if i < intercept_index:
        # Simple linear interpolation towards Mars (for illustration only)
        frac = i / intercept_index
        pos = (1 - frac) * earth_positions[launch_index] + frac * mars_positions[intercept_index]
    elif i < return_index:
        # Stay at Mars (illustrative placeholder)
        pos = mars_positions[intercept_index]
    else:
        # Simple linear return to Earth
        frac = (i - return_index) / (len(times) - return_index)
        pos = (1 - frac) * mars_positions[intercept_index] + frac * earth_positions[0]  # Return to initial Earth position
    spacecraft_positions.append(pos)

spacecraft_positions = np.array(spacecraft_positions)

# Plot initialization
earth_line, = ax.plot([], [], [], 'bo', label='Earth')
mars_line, = ax.plot([], [], [], 'ro', label='Mars')
spacecraft_line, = ax.plot([], [], [], 'g-', label='Spacecraft')

ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.5 * AU, 0.5 * AU])
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.legend()

# Update function for animation
def update(num, earth_positions, mars_positions, spacecraft_positions, earth_line, mars_line, spacecraft_line):
    earth_line.set_data(earth_positions[:num, 0], earth_positions[:num, 1])
    earth_line.set_3d_properties(earth_positions[:num, 2])
    
    mars_line.set_data(mars_positions[:num, 0], mars_positions[:num, 1])
    mars_line.set_3d_properties(mars_positions[:num, 2])
    
    spacecraft_line.set_data(spacecraft_positions[:num, 0], spacecraft_positions[:num, 1])
    spacecraft_line.set_3d_properties(spacecraft_positions[:num, 2])
    
    return earth_line, mars_line, spacecraft_line

# Create animation
ani = FuncAnimation(fig, update, frames=len(times), 
                    fargs=(earth_positions, mars_positions, spacecraft_positions, earth_line, mars_line, spacecraft_line),
                    interval=50, blit=False)

plt.show()