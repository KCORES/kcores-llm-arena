import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e11  # Astronomical Unit (m)
EARTH_RADIUS = 6.371e6  # Earth radius (m)
MARS_RADIUS = 3.389e6  # Mars radius (m)
TIME_STEP = 10  # Time step for animation (days)
TOTAL_DAYS = 800  # Total simulation time (days)
LAUNCH_WINDOW_DELAY = 26 * 30  # Next launch window delay (days, ~26 months)

# Function to calculate positions (simplified elliptical orbits)
def calculate_orbit(radius, angular_speed, t):
    theta = angular_speed * t
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = 0  # Assuming coplanar orbits for simplicity
    return x, y, z

# Generate Earth and Mars positions
days = np.arange(0, TOTAL_DAYS, TIME_STEP)
earth_orbit_radius = AU
mars_orbit_radius = 1.524 * AU  # Mars is 1.524 AU from the Sun

earth_positions = np.array([calculate_orbit(earth_orbit_radius, 2 * np.pi / (365), t) for t in days])
mars_positions = np.array([calculate_orbit(mars_orbit_radius, 2 * np.pi / (687), t) for t in days])

# Launch and landing points
launch_time = 0
landing_time = 400  # Simulated landing time (days)
return_time = landing_time + 300  # Simulated return time (days)

# Spacecraft trajectory (simplified straight-line interpolation)
def spacecraft_trajectory(start, end, duration, total_days):
    t = np.arange(0, total_days, TIME_STEP)
    trajectory = []
    for ti in t:
        if ti <= launch_time:
            point = start
        elif ti <= landing_time:
            progress = (ti - launch_time) / (landing_time - launch_time)
            point = (1 - progress) * np.array(start) + progress * np.array(end)
        elif ti <= return_time:
            progress = (ti - landing_time) / (return_time - landing_time)
            point = (1 - progress) * np.array(end) + progress * np.array(start)
        else:
            point = start
        trajectory.append(point)
    return np.array(trajectory)

# Earth to Mars trajectory
earth_to_mars_trajectory = spacecraft_trajectory(
    earth_positions[launch_time], mars_positions[landing_time], landing_time - launch_time, TOTAL_DAYS
)

# Mars to Earth trajectory
mars_to_earth_trajectory = spacecraft_trajectory(
    mars_positions[landing_time], earth_positions[return_time], return_time - landing_time, TOTAL_DAYS
)

# Combine all trajectories
spacecraft_positions = np.vstack([
    earth_positions[:launch_time],
    earth_to_mars_trajectory,
    mars_positions[landing_time:return_time],
    mars_to_earth_trajectory,
    earth_positions[return_time:]
])

# Plotting
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Draw orbits
earth_orbit = ax.plot(earth_positions[:, 0], earth_positions[:, 1], earth_positions[:, 2], 'b', label='Earth Orbit')[0]
mars_orbit = ax.plot(mars_positions[:, 0], mars_positions[:, 1], mars_positions[:, 2], 'r', label='Mars Orbit')[0]

# Draw Earth and Mars
earth = ax.scatter([earth_positions[0, 0]], [earth_positions[0, 1]], [earth_positions[0, 2]], c='blue', s=100, label='Earth')
mars = ax.scatter([mars_positions[0, 0]], [mars_positions[0, 1]], [mars_positions[0, 2]], c='red', s=80, label='Mars')

# Initialize spacecraft marker
spacecraft = ax.scatter([], [], [], c='green', s=50, label='Spacecraft')

# Animation function
def update(frame):
    ax.clear()
    
    # Draw orbits
    ax.plot(earth_positions[:, 0], earth_positions[:, 1], earth_positions[:, 2], 'b', label='Earth Orbit')
    ax.plot(mars_positions[:, 0], mars_positions[:, 1], mars_positions[:, 2], 'r', label='Mars Orbit')
    
    # Draw Earth and Mars
    ax.scatter([earth_positions[frame, 0]], [earth_positions[frame, 1]], [earth_positions[frame, 2]], c='blue', s=100, label='Earth')
    ax.scatter([mars_positions[frame, 0]], [mars_positions[frame, 1]], [mars_positions[frame, 2]], c='red', s=80, label='Mars')
    
    # Update spacecraft position
    if frame < launch_time:
        spacecraft_pos = earth_positions[frame]
    elif frame < landing_time:
        progress = (frame - launch_time) / (landing_time - launch_time)
        spacecraft_pos = (1 - progress) * earth_positions[launch_time] + progress * mars_positions[landing_time]
    elif frame < return_time:
        progress = (frame - landing_time) / (return_time - landing_time)
        spacecraft_pos = (1 - progress) * mars_positions[landing_time] + progress * earth_positions[return_time]
    else:
        spacecraft_pos = earth_positions[frame]
    
    spacecraft = ax.scatter([spacecraft_pos[0]], [spacecraft_pos[1]], [spacecraft_pos[2]], c='green', s=50, label='Spacecraft')
    
    # Set plot limits
    ax.set_xlim([-2 * AU, 2 * AU])
    ax.set_ylim([-2 * AU, 2 * AU])
    ax.set_zlim([-AU, AU])
    
    # Labels and legend
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_title('Spacecraft Trajectory: Earth to Mars and Back')

# Create animation
ani = FuncAnimation(fig, update, frames=len(days), interval=50, repeat=True)

# Show plot
plt.show()