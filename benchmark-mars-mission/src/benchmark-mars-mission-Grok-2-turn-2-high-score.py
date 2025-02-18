import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
G = 6.67430e-11  # Gravitational constant
M_sun = 1.989e30  # Mass of Sun
AU = 1.496e11  # Astronomical Unit in meters
day = 86400  # seconds in a day

# Orbital parameters
r_earth = 1 * AU  # Earth's average distance from the Sun
r_mars = 1.524 * AU  # Mars' average distance from the Sun

# Calculate Hohmann transfer properties
a_transfer = (r_earth + r_mars) / 2  # Semi-major axis of transfer orbit
T_transfer = 2 * np.pi * np.sqrt(a_transfer**3 / (G * M_sun))  # Time for transfer

# Time vectors
t = np.linspace(0, T_transfer, 1000)  # Outbound trip
t_return = np.linspace(T_transfer, T_transfer * 2, 1000)  # Return trip

# Position vectors for the spacecraft
theta_earth = 0  # Start at Earth
theta_transfer = np.sqrt(G * M_sun / a_transfer**3) * t
x_transfer = a_transfer * (np.cos(theta_transfer) - np.cos(theta_earth))
y_transfer = a_transfer * np.sin(theta_transfer)

theta_mars = (
    np.sqrt(G * M_sun / r_mars**3) * T_transfer
)  # Mars' angular position after transfer
theta_return = np.sqrt(G * M_sun / a_transfer**3) * t_return + theta_mars
x_return = a_transfer * (np.cos(theta_return) - np.cos(theta_mars))
y_return = a_transfer * np.sin(theta_return)

# Combine outbound and return paths
x_spacecraft = np.concatenate((x_transfer, x_return))
y_spacecraft = np.concatenate((y_transfer, y_return))

# Set up the plot
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Earth and Mars orbits
theta = np.linspace(0, 2 * np.pi, 1000)
x_earth = r_earth * np.cos(theta)
y_earth = r_earth * np.sin(theta)
x_mars = r_mars * np.cos(theta)
y_mars = r_mars * np.sin(theta)

# Plot static orbits
(earth_orbit,) = ax.plot(x_earth, y_earth, zs=0, zdir="z", label="Earth Orbit")
(mars_orbit,) = ax.plot(x_mars, y_mars, zs=0, zdir="z", label="Mars Orbit")

# Initial spacecraft position
(spacecraft,) = ax.plot(
    [x_spacecraft[0]],
    [y_spacecraft[0]],
    zs=[0],
    marker="o",
    color="r",
    label="Spacecraft",
)

ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.1 * AU, 0.1 * AU])
ax.legend()


# Animation function
def animate(i):
    spacecraft.set_data(x_spacecraft[:i], y_spacecraft[:i])
    spacecraft.set_3d_properties([0] * i)
    return (spacecraft,)


# Create animation
anim = FuncAnimation(fig, animate, frames=len(x_spacecraft), interval=50, blit=False)

plt.show()
