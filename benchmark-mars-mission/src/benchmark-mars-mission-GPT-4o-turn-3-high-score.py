import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the positions of Earth and Mars
earth_position = np.array([0, 0, 0])
mars_position = np.array([1.5, 0, 0])  # Simplified position


# Define the trajectory
def trajectory(t):
    # Simple circular trajectory for demonstration
    theta = np.linspace(0, 2 * np.pi, 100)
    x = np.cos(theta + t)
    y = np.sin(theta + t)
    z = np.sin(2 * theta + t) * 0.5
    return x, y, z


# Create the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Initialize the plot
(line,) = ax.plot([], [], [], "b-", lw=2)
(point,) = ax.plot([], [], [], "ro")

# Set the limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")


# Animation function
def update(frame):
    x, y, z = trajectory(frame / 10.0)
    line.set_data(x, y)
    line.set_3d_properties(z)
    point.set_data(x[-1:], y[-1:])
    point.set_3d_properties(z[-1:])
    return line, point


# Create the animation
ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)

# Show the plot
plt.show()
