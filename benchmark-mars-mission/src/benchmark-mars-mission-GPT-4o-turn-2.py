import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the orbital parameters
earth_radius = 6371  # in km
mars_radius = 3389.5  # in km
distance_earth_mars = 225000000  # average distance in km

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot Earth
earth = plt.Circle((0, 0), earth_radius, color="b", alpha=0.5)
ax.add_patch(earth)
art3d.pathpatch_2d_to_3d(earth, z=0, zdir="z")

# Plot Mars
mars = plt.Circle((distance_earth_mars, 0), mars_radius, color="r", alpha=0.5)
ax.add_patch(mars)
art3d.pathpatch_2d_to_3d(mars, z=0, zdir="z")

# Set the limits of the plot
ax.set_xlim(-distance_earth_mars * 0.1, distance_earth_mars * 1.1)
ax.set_ylim(-distance_earth_mars * 0.1, distance_earth_mars * 0.1)
ax.set_zlim(-distance_earth_mars * 0.1, distance_earth_mars * 0.1)

# Define the spacecraft trajectory
num_points = 100
trajectory = np.linspace(0, distance_earth_mars, num_points)
x = trajectory
y = np.sin(trajectory / distance_earth_mars * 2 * np.pi) * distance_earth_mars * 0.1
z = np.cos(trajectory / distance_earth_mars * 2 * np.pi) * distance_earth_mars * 0.1

# Plot the initial position of the spacecraft
(spacecraft,) = ax.plot([], [], [], "go", markersize=5)


# Animation function
def update(num):
    spacecraft.set_data(x[num], y[num])
    spacecraft.set_3d_properties(z[num])
    return (spacecraft,)


# Create the animation
ani = FuncAnimation(fig, update, frames=num_points, interval=50, blit=True)

# Show the plot
plt.show()
