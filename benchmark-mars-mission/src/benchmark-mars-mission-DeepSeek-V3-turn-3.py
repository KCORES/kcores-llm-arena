import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 149597870.7  # Astronomical Unit in kilometers
earth_orbit_radius = AU  # Earth's average distance from the Sun
mars_orbit_radius = 1.52 * AU  # Mars' average distance from the Sun
earth_orbital_period = 365.25  # Earth's orbital period in days
mars_orbital_period = 687  # Mars' orbital period in days


# Function to calculate the position of a planet in its orbit
def planet_position(radius, period, t):
    theta = 2 * np.pi * t / period
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return x, y, 0  # z=0 for simplicity (planets in the same plane)


# Function to calculate the spacecraft's trajectory
def spacecraft_trajectory(t_out, t_in, t_return):
    x_out = np.linspace(earth_orbit_radius, mars_orbit_radius, t_out)
    x_in = np.linspace(mars_orbit_radius, earth_orbit_radius, t_in)
    x_return = np.linspace(earth_orbit_radius, mars_orbit_radius, t_return)

    y_out = np.linspace(0, 0, t_out)
    y_in = np.linspace(0, 0, t_in)
    y_return = np.linspace(0, 0, t_return)

    z_out = np.linspace(0, 0, t_out)
    z_in = np.linspace(0, 0, t_in)
    z_return = np.linspace(0, 0, t_return)

    return (
        np.concatenate([x_out, x_in, x_return]),
        np.concatenate([y_out, y_in, y_return]),
        np.concatenate([z_out, z_in, z_return]),
    )


# Animation function
def animate(i):
    ax.cla()
    ax.set_xlim([-2 * AU, 2 * AU])
    ax.set_ylim([-2 * AU, 2 * AU])
    ax.set_zlim([-0.5 * AU, 0.5 * AU])

    # Earth's position
    earth_x, earth_y, earth_z = planet_position(
        earth_orbit_radius, earth_orbital_period, i
    )
    ax.scatter(earth_x, earth_y, earth_z, color="blue", label="Earth")

    # Mars' position
    mars_x, mars_y, mars_z = planet_position(mars_orbit_radius, mars_orbital_period, i)
    ax.scatter(mars_x, mars_y, mars_z, color="red", label="Mars")

    # Spacecraft trajectory
    sc_x, sc_y, sc_z = spacecraft_trajectory(100, 100, 100)
    ax.plot(sc_x[:i], sc_y[:i], sc_z[:i], color="green", label="Spacecraft")

    ax.legend()
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")


# Create the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Create the animation
ani = FuncAnimation(fig, animate, frames=300, interval=50)

# Display the animation
plt.show()
