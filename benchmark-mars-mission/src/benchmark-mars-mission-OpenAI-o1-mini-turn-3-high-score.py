import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def simulate_launch():
    # Constants
    earth_radius = 6371  # in km
    mars_radius = 3389.5  # in km
    distance_earth_mars = 780000  # approximate distance in km
    launch_window_period = 780  # days

    # Create figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot Earth
    u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color="b", alpha=0.6)

    # Plot Mars
    x_mars = distance_earth_mars
    y_mars = 0
    z_mars = 0
    u, v = np.linspace(0, 2 * np.pi, 100), np.linspace(0, np.pi, 100)
    x = mars_radius * np.outer(np.cos(u), np.sin(v)) + x_mars
    y = mars_radius * np.outer(np.sin(u), np.sin(v)) + y_mars
    z = mars_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + z_mars
    ax.plot_surface(x, y, z, color="r", alpha=0.6)

    # Initialize spacecraft position
    (spacecraft,) = ax.plot([], [], [], "ko", markersize=5)

    def update(frame):
        # Simple linear trajectory for demonstration
        t = frame / 100
        # Launch to Mars
        if t < 0.5:
            x_pos = t * distance_earth_mars * 2
            y_pos = 0
            z_pos = earth_radius + t * (mars_radius + 1000)
        # Return to Earth
        else:
            x_pos = distance_earth_mars * 2 - (t - 0.5) * distance_earth_mars * 4
            y_pos = 0
            z_pos = mars_radius + 1000 - (t - 0.5) * (mars_radius + 1000 + earth_radius)
        spacecraft.set_data([x_pos], [y_pos])
        spacecraft.set_3d_properties([z_pos])
        return (spacecraft,)

    ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)
    plt.show()


if __name__ == "__main__":
    simulate_launch()
