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

# Plot Earth and Mars
earth = plt.Circle((0, 0), earth_radius, color="b", label="Earth")
mars = plt.Circle((distance_earth_mars, 0), mars_radius, color="r", label="Mars")

# Add Earth and Mars to the plot
ax.add_patch(earth)
ax.add_patch(mars)
ax.plot([0, distance_earth_mars], [0, 0], [0, 0], "k--")  # Line between Earth and Mars

# Set the limits of the plot
ax.set_xlim(-1.5 * distance_earth_mars, 1.5 * distance_earth_mars)
ax.set_ylim(-1.5 * distance_earth_mars, 1.5 * distance_earth_mars)
ax.set_zlim(-1.5 * distance_earth_mars, 1.5 * distance_earth_mars)


# Function to update the animation
def update(num, line, data):
    line.set_data(data[0:2, :num])
    line.set_3d_properties(data[2, :num])
    return (line,)


# Generate the trajectory data
num_points = 1000
t = np.linspace(0, 2 * np.pi, num_points)
x = distance_earth_mars * np.cos(t)
y = distance_earth_mars * np.sin(t)
z = np.sin(2 * t) * distance_earth_mars / 10

data = np.array([x, y, z])

# Create a line object
(line,) = ax.plot(data[0], data[1], data[2], label="Trajectory", color="g")

# Create the animation
ani = FuncAnimation(
    fig, update, num_points, fargs=(line, data), interval=50, blit=False
)

# Add labels and legend
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.legend()

# Show the plot
plt.show()
