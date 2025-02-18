import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Set up the figure and 3D axes
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")


# Create spheres for Earth and Mars
def create_sphere(radius, center, resolution=20):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    return x, y, z


# Create simplified orbital paths (using parametric equations)
def create_trajectory(start_pos, end_pos, height_factor=1.5, points=100):
    t = np.linspace(0, 1, points)
    mid_point = (start_pos + end_pos) / 2
    mid_point[1] = height_factor * np.linalg.norm(end_pos - start_pos) / 2

    # Create Bezier curve
    trajectory = (
        (1 - t)[:, np.newaxis] ** 2 * start_pos
        + 2 * (1 - t)[:, np.newaxis] * t[:, np.newaxis] * mid_point
        + t[:, np.newaxis] ** 2 * end_pos
    )
    return trajectory


# Define positions
earth_pos = np.array([0, 0, 0])
mars_pos = np.array([15, 0, 0])

# Create Earth and Mars
earth_radius = 1
mars_radius = 0.5
earth_x, earth_y, earth_z = create_sphere(earth_radius, earth_pos)
mars_x, mars_y, mars_z = create_sphere(mars_radius, mars_pos)

# Create trajectories
outbound_trajectory = create_trajectory(earth_pos, mars_pos)
return_trajectory = create_trajectory(mars_pos, earth_pos)


# Animation function
def animate(frame):
    ax.clear()

    # Plot Earth and Mars
    ax.plot_surface(earth_x, earth_y, earth_z, color="blue", alpha=0.8)
    ax.plot_surface(mars_x, mars_y, mars_z, color="red", alpha=0.8)

    # Plot trajectories
    total_frames = 200
    if frame < total_frames / 2:
        # Outbound journey
        current_point = frame / (total_frames / 2)
        idx = int(current_point * len(outbound_trajectory))
        ax.plot(
            outbound_trajectory[:idx, 0],
            outbound_trajectory[:idx, 1],
            outbound_trajectory[:idx, 2],
            "b--",
            alpha=0.5,
        )
        if idx > 0:
            ax.scatter(
                outbound_trajectory[idx - 1, 0],
                outbound_trajectory[idx - 1, 1],
                outbound_trajectory[idx - 1, 2],
                color="white",
                s=100,
            )
    else:
        # Return journey
        current_point = (frame - total_frames / 2) / (total_frames / 2)
        idx = int(current_point * len(return_trajectory))
        ax.plot(
            outbound_trajectory[:, 0],
            outbound_trajectory[:, 1],
            outbound_trajectory[:, 2],
            "b--",
            alpha=0.3,
        )
        ax.plot(
            return_trajectory[:idx, 0],
            return_trajectory[:idx, 1],
            return_trajectory[:idx, 2],
            "g--",
            alpha=0.5,
        )
        if idx > 0:
            ax.scatter(
                return_trajectory[idx - 1, 0],
                return_trajectory[idx - 1, 1],
                return_trajectory[idx - 1, 2],
                color="white",
                s=100,
            )

    # Set consistent view
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Earth-Mars Transfer Animation")
    ax.set_xlim([-5, 20])
    ax.set_ylim([-10, 10])
    ax.set_zlim([-10, 10])
    ax.view_init(elev=20, azim=frame)


# Create animation
anim = animation.FuncAnimation(fig, animate, frames=200, interval=50, blit=False)
plt.show()
