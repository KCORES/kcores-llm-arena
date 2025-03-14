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
# Launch from Earth
launch_date = datetime.datetime(2024, 10, 26)  # Example launch date
launch_speed = 11.2e3  # km/s to m/s (approx. escape velocity)
launch_angle_elevation = 45  # degrees
launch_angle_azimuth = 0  # degrees (East)

# Mars landing date (approx. 7-9 months later)
landing_date = datetime.datetime(2025, 7, 1)

# Return launch from Mars (next launch window)
return_launch_date = datetime.datetime(2026, 10, 26)

# --- Simulation Parameters ---
dt = 3600 * 24  # Time step (seconds) - 1 day
total_time = 365 * 24 * 3600 * 2  # Total simulation time (seconds) - 2 years
num_steps = int(total_time / dt)


# --- Helper Functions ---
def distance(x1, y1, z1, x2, y2, z2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)


def gravitational_force(m1, m2, r):
    return G * m1 * m2 / r**2


def unit_vector(x, y, z):
    mag = np.sqrt(x**2 + y**2 + z**2)
    if mag == 0:
        return 0, 0, 0
    return x / mag, y / mag, z / mag


# --- Initial Position and Velocity ---
# Earth-Mars positions (simplified, assuming circular orbits)
earth_distance = 1 * AU
mars_distance = 1.524 * AU

# Convert launch angle to radians
launch_angle_elevation_rad = np.radians(launch_angle_elevation)
launch_angle_azimuth_rad = np.radians(launch_angle_azimuth)

# Initial position (Earth surface)
x = R_earth
y = 0
z = 0

# Initial velocity
vx = (
    launch_speed * np.cos(launch_angle_elevation_rad) * np.cos(launch_angle_azimuth_rad)
)
vy = (
    launch_speed * np.cos(launch_angle_elevation_rad) * np.sin(launch_angle_azimuth_rad)
)
vz = launch_speed * np.sin(launch_angle_elevation_rad)

# --- Simulation ---
x_data = [x]
y_data = [y]
z_data = [z]
vx_data = [vx]
vy_data = [vy]
vz_data = [vz]

time = 0
current_date = launch_date

for i in range(num_steps):
    # Calculate distances to Earth and Mars
    r_earth = distance(x, y, z, 0, 0, 0)
    r_mars = distance(x, y, z, mars_distance, 0, 0)

    # Calculate gravitational forces
    F_earth = gravitational_force(M_earth, 1, r_earth)
    F_mars = gravitational_force(M_mars, 1, r_mars)

    # Calculate force vectors
    fx_earth, fy_earth, fz_earth = unit_vector(x, y, z)
    fx_mars, fy_mars, fz_mars = unit_vector(mars_distance - x, -y, -z)

    ax = F_earth * fx_earth + F_mars * fx_mars
    ay = F_earth * fy_earth + F_mars * fy_mars
    az = F_earth * fz_earth + F_mars * fz_mars

    # Update velocity
    vx += ax * dt
    vy += ay * dt
    vz += az * dt

    # Update position
    x += vx * dt
    y += vy * dt
    z += vz * dt

    # Store data
    x_data.append(x)
    y_data.append(y)
    z_data.append(z)
    vx_data.append(vx)
    vy_data.append(vy)
    vz_data.append(vz)

    time += dt
    current_date += datetime.timedelta(days=1)

    # Check for Mars landing (simplified)
    if r_mars < R_mars and current_date >= landing_date:
        print(f"Landed on Mars on {current_date.strftime('%Y-%m-%d')}")
        break

    # Check for return launch (simplified)
    if current_date >= return_launch_date:
        print(f"Launching back to Earth on {current_date.strftime('%Y-%m-%d')}")
        break

# --- Plotting ---
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")

# Set axis limits (adjust as needed)
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-2 * AU, 2 * AU])

# Plot Earth and Mars
earth_sphere = plt.Sphere(radius=R_earth / 1e6, center=(0, 0, 0), color="blue")
mars_sphere = plt.Sphere(
    radius=R_mars / 1e6, center=(mars_distance / 1e6, 0, 0), color="red"
)
ax.add_collection3d(earth_sphere)
ax.add_collection3d(mars_sphere)

# Line for trajectory
(line,) = ax.plot([], [], [], lw=2)

# Text for date
date_text = ax.text(0.02, 0.95, "", transform=ax.transAxes)


def update(frame):
    line.set_data(x_data[:frame] / 1e6, y_data[:frame] / 1e6, z_data[:frame] / 1e6)
    current_date = launch_date + datetime.timedelta(
        days=frame / (num_steps / (total_time / (3600 * 24)))
    )
    date_text.set_text(f"Date: {current_date.strftime('%Y-%m-%d')}")
    return line, date_text


ani = FuncAnimation(fig, update, frames=num_steps, blit=True, repeat=False)

ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars Trajectory")

plt.show()
