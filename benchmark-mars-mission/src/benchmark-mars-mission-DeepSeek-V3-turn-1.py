import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e8  # Astronomical Unit in km
G = 6.67430e-11  # Gravitational constant in m^3 kg^-1 s^-2
M_sun = 1.989e30  # Mass of the Sun in kg
M_earth = 5.972e24  # Mass of Earth in kg
M_mars = 6.4171e23  # Mass of Mars in kg
r_earth = 6371  # Radius of Earth in km
r_mars = 3389.5  # Radius of Mars in km
r_earth_orbit = 1 * AU  # Earth's orbital radius in km
r_mars_orbit = 1.524 * AU  # Mars' orbital radius in km
T_earth = 365.25  # Earth's orbital period in days
T_mars = 687  # Mars' orbital period in days

# Time setup
days = np.linspace(0, 2 * T_earth, 500)  # Two Earth years

# Earth and Mars positions
earth_x = r_earth_orbit * np.cos(2 * np.pi * days / T_earth)
earth_y = r_earth_orbit * np.sin(2 * np.pi * days / T_earth)
earth_z = np.zeros_like(earth_x)

mars_x = r_mars_orbit * np.cos(2 * np.pi * days / T_mars)
mars_y = r_mars_orbit * np.sin(2 * np.pi * days / T_mars)
mars_z = np.zeros_like(mars_x)

# Spacecraft trajectory (Hohmann transfer)
transfer_time = 0.5 * (T_mars + T_earth)  # Approximate transfer time
transfer_days = np.linspace(0, transfer_time, 250)

transfer_x = r_earth_orbit * np.cos(2 * np.pi * transfer_days / T_earth) + (
    r_mars_orbit - r_earth_orbit
) * np.cos(2 * np.pi * transfer_days / transfer_time)
transfer_y = r_earth_orbit * np.sin(2 * np.pi * transfer_days / T_earth) + (
    r_mars_orbit - r_earth_orbit
) * np.sin(2 * np.pi * transfer_days / transfer_time)
transfer_z = np.zeros_like(transfer_x)

# Return trajectory
return_days = np.linspace(transfer_time, 2 * transfer_time, 250)
return_x = r_mars_orbit * np.cos(2 * np.pi * return_days / T_mars) + (
    r_earth_orbit - r_mars_orbit
) * np.cos(2 * np.pi * (return_days - transfer_time) / transfer_time)
return_y = r_mars_orbit * np.sin(2 * np.pi * return_days / T_mars) + (
    r_earth_orbit - r_mars_orbit
) * np.sin(2 * np.pi * (return_days - transfer_time) / transfer_time)
return_z = np.zeros_like(return_x)

# Combine trajectories
spacecraft_x = np.concatenate((transfer_x, return_x))
spacecraft_y = np.concatenate((transfer_y, return_y))
spacecraft_z = np.concatenate((transfer_z, return_z))

# Plot setup
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-2 * AU, 2 * AU])
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")

# Plot Earth and Mars orbits
ax.plot(earth_x, earth_y, earth_z, label="Earth's Orbit", color="blue")
ax.plot(mars_x, mars_y, mars_z, label="Mars's Orbit", color="red")

# Initialize spacecraft trajectory
(spacecraft_line,) = ax.plot([], [], [], label="Spacecraft Trajectory", color="green")


# Animation function
def update(frame):
    spacecraft_line.set_data(spacecraft_x[:frame], spacecraft_y[:frame])
    spacecraft_line.set_3d_properties(spacecraft_z[:frame])
    return (spacecraft_line,)


# Create animation
ani = FuncAnimation(fig, update, frames=len(spacecraft_x), interval=50, blit=True)

# Display the animation
plt.legend()
plt.show()
