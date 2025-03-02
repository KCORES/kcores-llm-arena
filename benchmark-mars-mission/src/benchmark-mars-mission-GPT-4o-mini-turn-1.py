import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the positions of Earth and Mars
earth_pos = np.array([0, 0, 0])
mars_pos = np.array([1.5, 0, 0])  # Simplified position for Mars

# Create a figure and a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Set limits and labels
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])
ax.set_xlabel("X axis")
ax.set_ylabel("Y axis")
ax.set_zlabel("Z axis")

# Initialize the plot with Earth and Mars
(earth,) = ax.plot(
    [earth_pos[0]], [earth_pos[1]], [earth_pos[2]], "bo", markersize=10, label="Earth"
)
(mars,) = ax.plot(
    [mars_pos[0]], [mars_pos[1]], [mars_pos[2]], "ro", markersize=10, label="Mars"
)
(trajectory,) = ax.plot([], [], [], "g--", label="Trajectory")


# Animation function
def update(frame):
    # Calculate the trajectory points
    t = np.linspace(0, 1, 100)
    x = (1 - t) * earth_pos[0] + t * mars_pos[0]
    y = (1 - t) * earth_pos[1] + t * mars_pos[1]
    z = (
        (1 - t) * earth_pos[2] + t * mars_pos[2] + np.sin(t * np.pi) * 0.5
    )  # Add some height variation

    # Update the trajectory
    trajectory.set_data(x, y)
    trajectory.set_3d_properties(z)

    # Update the positions of Earth and Mars
    earth.set_data(earth_pos[0], earth_pos[1])
    earth.set_3d_properties(earth_pos[2])
    mars.set_data(mars_pos[0], mars_pos[1])
    mars.set_3d_properties(mars_pos[2])

    return earth, mars, trajectory


# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), blit=True)

# Show the plot
plt.legend()
plt.show()
