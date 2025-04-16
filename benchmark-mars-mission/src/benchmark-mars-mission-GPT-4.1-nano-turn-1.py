import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
earth_pos = np.array([0, 0, 0])
mars_pos = np.array([1.5, 0, 0])  # Approximate distance in arbitrary units
launch_duration = 100  # frames from Earth to Mars
return_duration = 100  # frames from Mars back to Earth
total_duration = launch_duration + return_duration


# Generate positions for the launch and return paths
# Linear interpolation between Earth and Mars
def generate_path(start, end, steps):
    return np.linspace(start, end, steps)


# Paths
path_earth_to_mars = generate_path(earth_pos, mars_pos, launch_duration)
path_mars_to_earth = generate_path(mars_pos, earth_pos, return_duration)

# Initialize plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")

# Set axes limits for better visualization
ax.set_xlim(-0.5, 2.0)
ax.set_ylim(-1.0, 1.0)
ax.set_zlim(-0.5, 1.0)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Earth to Mars and Back Launch Animation")

# Plot Earth and Mars
ax.plot(
    [earth_pos[0]],
    [earth_pos[1]],
    [earth_pos[2]],
    marker="o",
    color="blue",
    markersize=8,
    label="Earth",
)
ax.plot(
    [mars_pos[0]],
    [mars_pos[1]],
    [mars_pos[2]],
    marker="o",
    color="red",
    markersize=8,
    label="Mars",
)
ax.legend()

# Initialize spaceship marker
(spaceship,) = ax.plot(
    [], [], [], marker="o", color="green", markersize=6, label="Spaceship"
)


# Animation update function
def update(frame):
    if frame < launch_duration:
        # Traveling from Earth to Mars
        pos = path_earth_to_mars[frame]
    else:
        # Traveling from Mars back to Earth
        index = frame - launch_duration
        if index >= return_duration:
            index = return_duration - 1  # stay at Earth after return
        pos = path_mars_to_earth[index]

    # Update spaceship position
    spaceship.set_data(pos[0], pos[1])
    spaceship.set_3d_properties(pos[2])
    return (spaceship,)


# Create animation
ani = FuncAnimation(
    fig, update, frames=total_duration + 20, interval=50, blit=True, repeat=True
)

# Display the plot
plt.legend()
plt.show()
