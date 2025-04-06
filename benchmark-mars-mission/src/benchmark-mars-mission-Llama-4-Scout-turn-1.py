import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# Define constants
earth_radius = 6371  # km
mars_radius = 3389.5  # km
earth_orbit_radius = 42164  # km
mars_orbit_radius = 227939  # km

# Define launch and landing points
launch_point = np.array([earth_radius, 0, 0])
landing_point = np.array(
    [
        mars_orbit_radius * np.cos(np.radians(45)),
        mars_orbit_radius * np.sin(np.radians(45)),
        0,
    ]
)
return_launch_point = np.array(
    [
        mars_orbit_radius * np.cos(np.radians(135)),
        mars_orbit_radius * np.sin(np.radians(135)),
        0,
    ]
)
return_landing_point = np.array([earth_radius, 0, 0])


# Define spacecraft's trajectory
def trajectory(t):
    if t < 10:
        # Launch from Earth to Mars orbit
        r = earth_radius + t * 5  # km
        theta = np.radians(45) * t / 10
        return np.array([r * np.cos(theta), r * np.sin(theta), 0])
    elif t < 40:
        # Mars orbit
        r = mars_orbit_radius
        theta = np.radians(45) + (t - 10) * np.radians(360) / 30
        return np.array([r * np.cos(theta), r * np.sin(theta), 0])
    elif t < 50:
        # Launch from Mars to Earth orbit
        r = mars_orbit_radius - (t - 40) * 5  # km
        theta = np.radians(135) + (t - 40) * np.radians(30) / 10
        return np.array([r * np.cos(theta), r * np.sin(theta), 0])
    else:
        # Return to Earth
        r = earth_orbit_radius - (t - 50) * 5  # km
        theta = np.radians(180) + (t - 50) * np.radians(180) / 20
        return np.array([r * np.cos(theta), r * np.sin(theta), 0])


# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Initialize plot
ax.set_xlim(-250000, 250000)
ax.set_ylim(-250000, 250000)
ax.set_zlim(-250000, 250000)

# Plot celestial bodies
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)
x_earth = earth_radius * np.outer(np.cos(u), np.sin(v))
y_earth = earth_radius * np.outer(np.sin(u), np.sin(v))
z_earth = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x_earth, y_earth, z_earth, color="b")

x_mars = mars_radius * np.outer(np.cos(u), np.sin(v)) + mars_orbit_radius * np.cos(
    np.radians(45)
)
y_mars = mars_radius * np.outer(np.sin(u), np.sin(v)) + mars_orbit_radius * np.sin(
    np.radians(45)
)
z_mars = mars_radius * np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x_mars, y_mars, z_mars, color="r")


# Animation function
def animate(i):
    ax.clear()
    ax.set_xlim(-250000, 250000)
    ax.set_ylim(-250000, 250000)
    ax.set_zlim(-250000, 250000)

    # Plot celestial bodies
    ax.plot_surface(x_earth, y_earth, z_earth, color="b")
    ax.plot_surface(x_mars, y_mars, z_mars, color="r")

    # Plot spacecraft's trajectory
    t = i / 10.0
    position = trajectory(t)
    ax.scatter(position[0], position[1], position[2], color="g")


ani = animation.FuncAnimation(
    fig, animate, frames=int(60 * 10), interval=100, repeat=False
)

plt.show()
