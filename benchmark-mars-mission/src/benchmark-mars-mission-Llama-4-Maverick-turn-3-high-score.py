import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
SUN_RADIUS = 0.2
EARTH_RADIUS = 0.05
MARS_RADIUS = 0.03
AU = 1.0  # Astronomical Unit (average distance between Earth and Sun)

# Orbital radii (simplified, assuming circular orbits)
R_EARTH = 1.0 * AU
R_MARS = 1.524 * AU

# Time parameters
T_EARTH = 365.25  # Days
T_MARS = 687.0  # Days
synodic_period = T_EARTH * T_MARS / abs(T_EARTH - T_MARS)

# Initial angles
theta_earth_initial = 0
theta_mars_initial = np.pi  # Mars is on the opposite side initially

# Trajectory parameters
num_points = 1000
t = np.linspace(0, synodic_period, num_points)

# Positions of Earth and Mars over time
theta_earth = theta_earth_initial + 2 * np.pi * t / T_EARTH
theta_mars = theta_mars_initial + 2 * np.pi * t / T_MARS

x_earth = R_EARTH * np.cos(theta_earth)
y_earth = R_EARTH * np.sin(theta_earth)
z_earth = np.zeros_like(t)

x_mars = R_MARS * np.cos(theta_mars)
y_mars = R_MARS * np.sin(theta_mars)
z_mars = np.zeros_like(t)

# Simplified spacecraft trajectory: Earth -> Mars -> Earth
# For simplicity, assume travel time is half the synodic period and the return journey starts after a short stay.
spacecraft_travel_time = synodic_period / 2
spacecraft_return_travel_time = synodic_period / 2
stay_time = 20  # days

# Departure and arrival indices
departure_idx = 0
arrival_at_mars_idx = int(spacecraft_travel_time / synodic_period * num_points)
departure_from_mars_idx = int(
    (spacecraft_travel_time + stay_time) / synodic_period * num_points
)
arrival_at_earth_idx = num_points - 1

# Spacecraft trajectory
x_spacecraft = np.zeros(num_points)
y_spacecraft = np.zeros(num_points)
z_spacecraft = np.zeros(num_points)

# Earth to Mars leg
for i in range(departure_idx, arrival_at_mars_idx):
    fraction = (i - departure_idx) / (arrival_at_mars_idx - departure_idx)
    x_spacecraft[i] = x_earth[i] + fraction * (x_mars[i] - x_earth[i])
    y_spacecraft[i] = y_earth[i] + fraction * (y_mars[i] - y_earth[i])

# Stay at Mars
for i in range(arrival_at_mars_idx, departure_from_mars_idx):
    x_spacecraft[i] = x_mars[i]
    y_spacecraft[i] = y_mars[i]

# Mars to Earth leg
for i in range(departure_from_mars_idx, arrival_at_earth_idx):
    fraction = (i - departure_from_mars_idx) / (
        arrival_at_earth_idx - departure_from_mars_idx
    )
    x_spacecraft[i] = x_mars[i] + fraction * (x_earth[i] - x_mars[i])
    y_spacecraft[i] = y_mars[i] + fraction * (y_earth[i] - y_mars[i])

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


# Animation function
def animate(i):
    ax.clear()
    ax.set_xlim(-R_MARS - 0.1, R_MARS + 0.1)
    ax.set_ylim(-R_MARS - 0.1, R_MARS + 0.1)
    ax.set_zlim(-R_MARS - 0.1, R_MARS + 0.1)

    # Plot Sun
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x_sun = SUN_RADIUS * np.outer(np.cos(u), np.sin(v))
    y_sun = SUN_RADIUS * np.outer(np.sin(u), np.sin(v))
    z_sun = SUN_RADIUS * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x_sun, y_sun, z_sun, color="yellow")

    # Plot Earth and Mars orbits
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot(
        R_EARTH * np.cos(theta),
        R_EARTH * np.sin(theta),
        np.zeros_like(theta),
        color="blue",
    )
    ax.plot(
        R_MARS * np.cos(theta),
        R_MARS * np.sin(theta),
        np.zeros_like(theta),
        color="red",
    )

    # Plot Earth and Mars positions
    ax.scatter(x_earth[i], y_earth[i], z_earth[i], color="blue", s=50)
    ax.scatter(x_mars[i], y_mars[i], z_mars[i], color="red", s=30)

    # Plot spacecraft trajectory up to the current point
    ax.plot(x_spacecraft[:i], y_spacecraft[:i], z_spacecraft[:i], color="green")
    ax.scatter(x_spacecraft[i], y_spacecraft[i], z_spacecraft[i], color="green", s=20)


ani = animation.FuncAnimation(fig, animate, frames=num_points, interval=20)

plt.show()
