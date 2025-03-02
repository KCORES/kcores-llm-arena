import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


# Define the trajectory
def trajectory(t):
    # Simple parabolic trajectory for demonstration
    x = 10 * t
    y = 5 * np.sin(t * np.pi)
    z = 10 * (1 - t)
    return x, y, z


# Create a figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Set limits
ax.set_xlim(0, 100)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 10)

# Create a point for the launch
(point,) = ax.plot([], [], [], "ro")


# Initialize the animation
def init():
    point.set_data([], [])
    point.set_3d_properties([])
    return (point,)


# Update the animation
def update(frame):
    t = frame / 100  # Normalize frame to [0, 1]
    x, y, z = trajectory(t)
    point.set_data(x, y)
    point.set_3d_properties(z)
    return (point,)


# Create the animation
ani = FuncAnimation(fig, update, frames=100, init_func=init, blit=True)

# Show the plot
plt.show()
