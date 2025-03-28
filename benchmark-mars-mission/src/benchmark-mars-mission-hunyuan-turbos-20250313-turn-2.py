import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e11  # Astronomical Unit (m)
earth_orbit_radius = 1 * AU
mars_orbit_radius = 1.524 * AU
earth_rotation_period = 365.25  # days
mars_rotation_period = 687  # days
time_step = 10  # days
total_time = (earth_rotation_period + mars_rotation_period) * 2  # Simulate one round trip

# Generate Earth and Mars orbits
theta_earth = np.linspace(0, 2 * np.pi, 100)
theta_mars = np.linspace(0, 2 * np.pi, 100)
x_earth = earth_orbit_radius * np.cos(theta_earth)
y_earth = earth_orbit_radius * np.sin(theta_earth)
z_earth = np.zeros_like(x_earth)

x_mars = mars_orbit_radius * np.cos(theta_mars)
y_mars = mars_orbit_radius * np.sin(theta_mars)
z_mars = np.zeros_like(x_mars)

# Launch and return path (simplified straight-line interpolation)
def get_spaceship_path(start, end, steps):
    x = np.linspace(start[0], end[0], steps)
    y = np.linspace(start[1], end[1], steps)
    z = np.linspace(start[2], end[2], steps)
    return x, y, z

# Earth to Mars
earth_to_mars_steps = 50
mars_landing_point = (mars_orbit_radius, 0, 0)  # Simplified landing on Mars
spaceship_path_earth_to_mars = get_spaceship_path(
    (earth_orbit_radius, 0, 0), mars_landing_point, earth_to_mars_steps
)

# Mars to Earth
mars_to_earth_steps = 50
spaceship_path_mars_to_earth = get_spaceship_path(
    mars_landing_point, (earth_orbit_radius, 0, 0), mars_to_earth_steps
)

# Combine paths
x_ship = np.concatenate([spaceship_path_earth_to_mars[0], mars_landing_point[0], spaceship_path_mars_to_earth[0]])
y_ship = np.concatenate([spaceship_path_earth_to_mars[1], mars_landing_point[1], spaceship_path_mars_to_earth[1]])
z_ship = np.concatenate([spaceship_path_earth_to_mars[2], mars_landing_point[2], spaceship_path_mars_to_earth[2]])

# Time array for animation
time = np.arange(0, total_time, time_step)

# Create 3D plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot Earth and Mars orbits
earth_orbit, = ax.plot(x_earth, y_earth, z_earth, label='Earth Orbit', color='blue')
mars_orbit, = ax.plot(x_mars, y_mars, z_mars, label='Mars Orbit', color='red')

# Initialize spaceship path
spaceship, = ax.plot([], [], [], label='Spaceship', color='green', marker='o')

# Labels and legend
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
ax.set_title('3D Animation of Earth-Mars-Earth Mission')

# Animation function
def update(frame):
    # Calculate spaceship position based on time
    if frame < len(spaceship_path_earth_to_mars[0]):
        # Earth to Mars
        ship_x = spaceship_path_earth_to_mars[0][frame]
        ship_y = spaceship_path_earth_to_mars[1][frame]
        ship_z = spaceship_path_earth_to_mars[2][frame]
    elif frame < len(spaceship_path_earth_to_mars[0]) + 1:
        # Mars landing
        ship_x, ship_y, ship_z = mars_landing_point
    else:
        # Mars to Earth
        offset = frame - (len(spaceship_path_earth_to_mars[0]) + 1)
        ship_x = spaceship_path_mars_to_earth[0][offset]
        ship_y = spaceship_path_mars_to_earth[1][offset]
        ship_z = spaceship_path_mars_to_earth[2][offset]

    # Update spaceship position
    spaceship.set_data([ship_x], [ship_y])
    spaceship.set_3d_properties([ship_z])
    return spaceship,

# Create animation
ani = FuncAnimation(fig, update, frames=np.arange(0, len(spaceship_path_earth_to_mars[0]) + 2 + mars_to_earth_steps), interval=50, blit=True)

# Show plot
plt.show()