import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Set up the figure and 3D axes
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Create data for Earth and Mars orbits
# Simplified circular/elliptical orbits
t = np.linspace(0, 2 * np.pi, 100)
earth_radius = 10
mars_radius = 15

# Earth orbit
earth_x = earth_radius * np.cos(t)
earth_y = earth_radius * np.sin(t)
earth_z = np.zeros_like(t)

# Mars orbit (slightly tilted)
mars_x = mars_radius * np.cos(t)
mars_y = mars_radius * np.sin(t)
mars_z = mars_radius * 0.1 * np.sin(t)  # Adding slight tilt


# Spacecraft trajectory (outbound)
def create_trajectory(start_pos, end_pos, height_factor=0.3):
    t_craft = np.linspace(0, 1, 50)
    x = start_pos[0] + (end_pos[0] - start_pos[0]) * t_craft
    y = start_pos[1] + (end_pos[1] - start_pos[1]) * t_craft
    z = height_factor * np.sin(np.pi * t_craft)
    return x, y, z


# Animation update function
def update(frame):
    ax.clear()

    # Plot orbits
    ax.plot(earth_x, earth_y, earth_z, "b-", alpha=0.3, label="Earth Orbit")
    ax.plot(mars_x, mars_y, mars_z, "r-", alpha=0.3, label="Mars Orbit")

    # Current positions of planets
    earth_pos = (earth_x[frame], earth_y[frame], earth_z[frame])
    mars_pos = (mars_x[frame], mars_y[frame], mars_z[frame])

    # Plot planets
    ax.scatter(*earth_pos, color="blue", s=100, label="Earth")
    ax.scatter(*mars_pos, color="red", s=80, label="Mars")

    # Spacecraft trajectories
    if frame < 50:
        # Outbound journey
        craft_x, craft_y, craft_z = create_trajectory(earth_pos, mars_pos)
        progress = frame / 50
        current_pos = (
            craft_x[int(progress * len(craft_x))],
            craft_y[int(progress * len(craft_y))],
            craft_z[int(progress * len(craft_z))],
        )
    else:
        # Return journey
        craft_x, craft_y, craft_z = create_trajectory(mars_pos, earth_pos)
        progress = (frame - 50) / 50
        current_pos = (
            craft_x[int(progress * len(craft_x))],
            craft_y[int(progress * len(craft_y))],
            craft_z[int(progress * len(craft_z))],
        )

    # Plot spacecraft
    ax.scatter(*current_pos, color="green", s=50, label="Spacecraft")

    # Setting plot limits and labels
    ax.set_xlim([-20, 20])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-5, 5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    ax.set_title(f"Earth-Mars Mission Simulation (Frame {frame})")


# Create animation
anim = animation.FuncAnimation(fig, update, frames=100, interval=100)

plt.show()
