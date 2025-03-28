import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e11  # Astronomical Unit (m)
EARTH_RADIUS = 6.371e6  # Earth radius (m)
MARS_RADIUS = 3.389e6  # Mars radius (m)
EARTH_ORBIT_RADIUS = 1 * AU
MARS_ORBIT_RADIUS = 1.524 * AU
TIME_STEP = 10  # Days per frame
SIMULATION_DAYS = 730  # Total simulation time (2 years)

# Helper function to calculate positions in orbits
def circular_orbit_position(radius, angle):
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0
    return x, y, z

# Generate Earth and Mars positions
earth_angles = np.linspace(0, 2 * np.pi, SIMULATION_DAYS // TIME_STEP)
mars_angles = np.linspace(0, 2 * np.pi + 0.5 * np.pi, SIMULATION_DAYS // TIME_STEP)  # Mars leads by 240 degrees

earth_positions = np.array([circular_orbit_position(EARTH_ORBIT_RADIUS, angle) for angle in earth_angles])
mars_positions = np.array([circular_orbit_position(MARS_ORBIT_RADIUS, angle) for angle in mars_angles])

# Launch and landing parameters
launch_duration = 10  # Days
landing_duration = 5  # Days
return_duration = 200  # Days

# Interpolation for the spacecraft trajectory
def interpolate_trajectory(start, end, duration, steps):
    t = np.linspace(0, 1, steps)
    x = start[0] + t * (end[0] - start[0])
    y = start[1] + t * (end[1] - start[1])
    z = start[2] + t * (end[2] - start[2])
    return x, y, z

# Spacecraft trajectory
spacecraft_trajectory = []

# Launch from Earth
launch_start = earth_positions[0]
launch_end = earth_positions[launch_duration // TIME_STEP]
x_launch, y_launch, z_launch = interpolate_trajectory(launch_start, launch_end, launch_duration, 50)
spacecraft_trajectory.append((x_launch, y_launch, z_launch))

# Transit to Mars
transit_start = launch_end
transit_end = mars_positions[launch_duration // TIME_STEP]
x_transit, y_transit, z_transit = interpolate_trajectory(transit_start, transit_end, 300, 100)
spacecraft_trajectory.append((x_transit, y_transit, z_transit))

# Landing on Mars
landing_start = transit_end
landing_end = mars_positions[launch_duration // TIME_STEP + landing_duration // TIME_STEP]
x_landing, y_landing, z_landing = interpolate_trajectory(landing_start, landing_end, landing_duration, 20)
spacecraft_trajectory.append((x_landing, y_landing, z_landing))

# Stay on Mars (simulated as a stationary point)
stay_start = landing_end
stay_end = stay_start
x_stay, y_stay, z_stay = np.full((50, 1), stay_start[0]), np.full((50, 1), stay_start[1]), np.full((50, 1), stay_start[2])
spacecraft_trajectory.append((x_stay, y_stay, z_stay))

# Return to Earth
return_start = stay_end[-1]
return_end = earth_positions[-1]
x_return, y_return, z_return = interpolate_trajectory(return_start, return_end, return_duration, 100)
spacecraft_trajectory.append((x_return, y_return, z_return))

# Combine all trajectory segments
trajectory = np.concatenate([
    np.column_stack(t) for t in spacecraft_trajectory
], axis=0)

# Animation setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot Earth and Mars orbits
earth_orbit, = ax.plot([], [], [], 'b', label='Earth Orbit')
mars_orbit, = ax.plot([], [], [], 'r', label='Mars Orbit')

# Plot Earth and Mars
earth = ax.scatter([], [], [], c='blue', s=EARTH_RADIUS**2 / 1e6, label='Earth')
mars = ax.scatter([], [], [], c='red', s=MARS_RADIUS**2 / 1e6, label='Mars')

# Plot spacecraft
spacecraft, = ax.plot([], [], [], 'k-o', label='Spacecraft')

# Labels and legend
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.legend()
ax.set_title('3D Animation of Earth-Mars-Earth Mission')

# Animation update function
def update(frame):
    # Update Earth and Mars orbits
    earth_orbit.set_data(earth_positions[:frame, 0], earth_positions[:frame, 1])
    earth_orbit.set_3d_properties(earth_positions[:frame, 2])
    mars_orbit.set_data(mars_positions[:frame, 0], mars_positions[:frame, 1])
    mars_orbit.set_3d_properties(mars_positions[:frame, 2])

    # Update Earth and Mars positions
    earth._offsets3d = (earth_positions[frame, 0:1], earth_positions[frame, 1:2], np.zeros(1))
    mars._offsets3d = (mars_positions[frame, 0:1], mars_positions[frame, 1:2], np.zeros(1))

    # Update spacecraft trajectory
    if frame < len(trajectory):
        spacecraft.set_data(trajectory[:frame, 0], trajectory[:frame, 1])
        spacecraft.set_3d_properties(trajectory[:frame, 2])

    return earth_orbit, mars_orbit, earth, mars, spacecraft

# Create animation
ani = FuncAnimation(fig, update, frames=range(len(earth_positions)), interval=50, blit=False)

# Show plot
plt.show()