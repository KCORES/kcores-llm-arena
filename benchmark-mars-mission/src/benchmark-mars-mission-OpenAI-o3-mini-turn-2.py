import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def generate_bezier(P0, P1, P2, num_points=100):
    """
    Generate num_points points along a quadratic Bezier curve defined by control points P0, P1, and P2.
    """
    t = np.linspace(0, 1, num_points)
    curve = (
        (1 - t) ** 2[:, None] * P0 + 2 * (1 - t) * t[:, None] * P1 + t**2[:, None] * P2
    )
    return curve


def plot_sphere(ax, center, radius, color, alpha=0.6, resolution=20):
    """
    Plot a sphere on a given 3D axes.
    """
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    ax.plot_surface(
        x, y, z, color=color, alpha=alpha, rstride=1, cstride=1, linewidth=0
    )


def main():
    # Define planet positions and radii
    earth_center = np.array([0, 0, 0])
    mars_center = np.array([15, 0, 0])
    earth_radius = 1.0
    mars_radius = 0.5

    # Generate the trajectory segments using quadratic Bezier curves
    # Segment 1: from Earth to Mars with a curve (launch phase)
    cp1 = np.array([7.5, 8, 8])  # Control point for first segment (arc upward)
    trajectory1 = generate_bezier(earth_center, cp1, mars_center, num_points=100)

    # Pause at Mars: replicate Mars position to simulate landing/waiting phase
    pause_frames = 50
    trajectory_pause = np.tile(mars_center, (pause_frames, 1))

    # Segment 2: from Mars back to Earth with a different curve (return phase)
    cp2 = np.array([7.5, -8, 8])  # Control point for second segment (arc downward)
    trajectory2 = generate_bezier(mars_center, cp2, earth_center, num_points=100)

    # Combine all trajectory points
    trajectory = np.concatenate((trajectory1, trajectory_pause, trajectory2), axis=0)
    total_frames = trajectory.shape[0]

    # Setup figure and 3D axis
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Plot Earth and Mars spheres
    plot_sphere(ax, earth_center, earth_radius, color="b", alpha=0.6)
    plot_sphere(ax, mars_center, mars_radius, color="r", alpha=0.6)

    # Optionally, plot the full trajectory as a dashed line for reference
    ax.plot(
        trajectory[:, 0],
        trajectory[:, 1],
        trajectory[:, 2],
        "k--",
        linewidth=1,
        alpha=0.5,
    )

    # Create a spaceship marker and a line that will show the path traveled so far
    (spaceship,) = ax.plot([], [], [], "o", color="g", markersize=8)  # spaceship marker
    (path_line,) = ax.plot([], [], [], "-", color="g", linewidth=2)  # trajectory line

    # Setting the axes limits
    ax.set_xlim(-2, 17)
    ax.set_ylim(-12, 12)
    ax.set_zlim(-2, 12)

    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Launch from Earth to Mars and Return at Next Launch Window")

    def update(frame):
        # Update spaceship position
        pos = trajectory[frame]
        spaceship.set_data(pos[0], pos[1])
        spaceship.set_3d_properties(pos[2])

        # Update the trajectory line to show the path traveled so far
        path_line.set_data(trajectory[: frame + 1, 0], trajectory[: frame + 1, 1])
        path_line.set_3d_properties(trajectory[: frame + 1, 2])

        return spaceship, path_line

    # Create animation; blit=False is used here as 3D animations may not support blitting well.
    anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)

    plt.show()


if __name__ == "__main__":
    main()
