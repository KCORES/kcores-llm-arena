import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import datetime

# --- Constants ---
G = 6.6743e-11  # Gravitational constant
M_earth = 5.972e24  # Mass of Earth (kg)
M_mars = 6.417e23  # Mass of Mars (kg)
R_earth = 6371e3  # Radius of Earth (m)
R_mars = 3389.5e3  # Radius of Mars (m)
AU = 1.496e11  # Astronomical Unit (m)

# --- Initial Conditions ---
# Earth and Mars positions (simplified, assuming circular orbits)
# These are approximate, and a more accurate ephemeris would be needed for a real simulation
earth_distance = 1 * AU
mars_distance = 1.524 * AU  # Mars' average distance from the sun

# Launch parameters
launch_speed = 11.2e3  # km/s to m/s (approx. Earth escape velocity)
launch_angle_elevation = 45  # degrees
launch_angle_azimuth = 0  # degrees (relative to some reference)

# Simulation parameters
dt = 3600 * 24  # Time step (seconds - one day)
total_time = 3600 * 24 * 400  # Total simulation time (seconds - approx. 1.1 years)
num_frames = int(total_time / dt)

# --- Helper Functions ---

def distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def gravitational_force(pos1, pos2, m1, m2):
    r = distance(pos1, pos2)
    force_magnitude = (G * m1 * m2) / (r**2)
    force_direction = (np.array(pos2) - np.array(pos1)) / r
    return force_magnitude * force_direction

# --- Initial State ---
# Spacecraft initial position and velocity
launch_date = datetime.datetime(2024, 1, 1)
launch_window_days = 260 # Approximate days to next launch window
next_launch_date = launch_date + datetime.timedelta(days=launch_window_days)

# Convert launch angle to radians
launch_angle_elevation_rad = np.radians(launch_angle_elevation)
launch_angle_azimuth_rad = np.radians(launch_angle_azimuth)

# Initial position (on Earth's surface)
initial_position = [R_earth, 0, 0]

# Initial velocity (launch velocity components)
vx = launch_speed * np.cos(launch_angle_elevation_rad) * np.cos(launch_angle_azimuth_rad)
vy = launch_speed * np.cos(launch_angle_elevation_rad) * np.sin(launch_angle_azimuth_rad)
vz = launch_speed * np.sin(launch_angle_elevation_rad)

initial_velocity = [vx, vy, vz]

# Earth and Mars initial positions (simplified)
earth_position = [0, 0, 0]
mars_position = [mars_distance, 0, 0]

# --- Simulation ---
positions = []
current_position = initial_position
current_velocity = initial_velocity

for _ in range(num_frames):
    # Calculate gravitational forces
    force_earth = gravitational_force(current_position, earth_position, M_mars, M_earth) # Spacecraft to Earth
    force_mars = gravitational_force(current_position, mars_position, M_earth, M_mars) # Spacecraft to Mars

    # Net force
    net_force = force_earth + force_mars

    # Calculate acceleration
    acceleration = net_force / 1000  # Assuming spacecraft mass of 1000 kg

    # Update velocity
    current_velocity = [current_velocity[i] + acceleration[i] * dt for i in range(3)]

    # Update position
    current_position = [current_position[i] + current_velocity[i] * dt for i in range(3)]

    positions.append(current_position.copy())

    # Check for landing on Mars (simplified)
    if distance(current_position, mars_position) <= R_mars:
        print("Landed on Mars!")
        break

    # Check for landing on Earth (simplified)
    if distance(current_position, earth_position) <= R_earth:
        print("Landed on Earth!")
        break

positions = np.array(positions)

# --- Plotting ---
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection='3d')

# Set axis limits
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-2 * AU, 2 * AU])

# Plot Earth and Mars
earth_sphere = ax.plot_surface(
    (R_earth * np.sin(np.linspace(0, 2 * np.pi, 50)) * np.cos(np.linspace(0, np.pi, 50))),
    (R_earth * np.sin(np.linspace(0, 2 * np.pi, 50)) * np.sin(np.linspace(0, np.pi, 50))),
    (R_earth * np.cos(np.linspace(0, np.pi, 50))),
    color='blue', alpha=0.5
)

mars_sphere = ax.plot_surface(
    ((mars_distance + R_mars * np.sin(np.linspace(0, 2 * np.pi, 50)) * np.cos(np.linspace(0, np.pi, 50))),
    (mars_distance + R_mars * np.sin(np.linspace(0, 2 * np.pi, 50)) * np.sin(np.linspace(0, np.pi, 50))),
    (R_mars * np.cos(np.linspace(0, np.pi, 50))),
    color='red', alpha=0.5
)

# Trajectory line
trajectory, = ax.plot([], [], [], color='green', linewidth=1)

# Text for time
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# --- Animation Function ---
def animate(i):
    trajectory.set_data(positions[:i, 0], positions[:i, 1], positions[:i, 2])
    current_time = i * dt
    days = current_time / (3600 * 24)
    time_text.set_text(f'Time: {days:.1f} days')
    return trajectory, time_text

# --- Create Animation ---
ani = FuncAnimation(fig, animate, frames=len(positions), interval=20, blit=True)

# --- Show Plot ---
plt.title("Earth-Mars-Earth Trajectory")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
plt.show()
