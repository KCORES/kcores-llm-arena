import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def generate_trajectories(num_points=200):
    """
    Generate sample positions for Earth, Mars, and the spacecraft.
    This is a simplified example and not physically accurate.
    """
    # Time array
    t = np.linspace(0, 2 * np.pi, num_points)

    # Earth orbit (circular orbit around the sun, radius = 1)
    earth_x = np.cos(t)
    earth_y = np.sin(t)
    earth_z = np.zeros(num_points)

    # Mars orbit (circular orbit around the sun, larger radius = 1.5)
    mars_x = 1.5 * np.cos(t * 0.8)  # slower orbit
    mars_y = 1.5 * np.sin(t * 0.8)
    mars_z = np.zeros(num_points)

    # Spacecraft trajectory: Earth → Mars → Earth
    # (Simplified parametric path for demonstration)
    # Part 1: Earth to Mars
    part1_ratio = 0.5
    part1_end = int(num_points * part1_ratio)
    sc_x_1 = np.linspace(earth_x[0], mars_x[part1_end], part1_end)
    sc_y_1 = np.linspace(earth_y[0], mars_y[part1_end], part1_end)
    sc_z_1 = 0.1 * np.sin(np.linspace(0, np.pi, part1_end))  # small arc

    # Part 2: Mars to Earth at next window
    sc_x_2 = np.linspace(mars_x[part1_end], earth_x[-1], num_points - part1_end)
    sc_y_2 = np.linspace(mars_y[part1_end], earth_y[-1], num_points - part1_end)
    sc_z_2 = 0.1 * np.sin(np.linspace(np.pi, 2 * np.pi, num_points - part1_end))

    sc_x = np.concatenate((sc_x_1, sc_x_2))
    sc_y = np.concatenate((sc_y_1, sc_y_2))
    sc_z = np.concatenate((sc_z_1, sc_z_2))

    return (earth_x, earth_y, earth_z), (mars_x, mars_y, mars_z), (sc_x, sc_y, sc_z)


def animate_launch():
    """
    Create and display the animated 3D plot showing
    the spacecraft traveling from Earth to Mars and back.
    """
    # Generate orbital and spacecraft data
    earth, mars, spacecraft = generate_trajectories()
    earth_x, earth_y, earth_z = earth
    mars_x, mars_y, mars_z = mars
    sc_x, sc_y, sc_z = spacecraft

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Set up the 3D space
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Earth-Mars Round Trip")

    # Plot Earth and Mars orbits
    (earth_orbit,) = ax.plot([], [], [], "b--", label="Earth Orbit")
    (mars_orbit,) = ax.plot([], [], [], "r--", label="Mars Orbit")

    # Plot Earth, Mars, and the spacecraft
    (earth_point,) = ax.plot([], [], [], "bo", label="Earth")
    (mars_point,) = ax.plot([], [], [], "ro", label="Mars")
    (spacecraft_point,) = ax.plot([], [], [], "go", label="Spacecraft")

    ax.legend()

    def init():
        earth_orbit.set_data([], [])
        earth_orbit.set_3d_properties([])

        mars_orbit.set_data([], [])
        mars_orbit.set_3d_properties([])

        earth_point.set_data([], [])
        earth_point.set_3d_properties([])

        mars_point.set_data([], [])
        mars_point.set_3d_properties([])

        spacecraft_point.set_data([], [])
        spacecraft_point.set_3d_properties([])

        return (earth_orbit, mars_orbit, earth_point, mars_point, spacecraft_point)

    def update(frame):
        earth_orbit.set_data(earth_x[:frame], earth_y[:frame])
        earth_orbit.set_3d_properties(earth_z[:frame])

        mars_orbit.set_data(mars_x[:frame], mars_y[:frame])
        mars_orbit.set_3d_properties(mars_z[:frame])

        earth_point.set_data(earth_x[frame], earth_y[frame])
        earth_point.set_3d_properties(earth_z[frame])

        mars_point.set_data(mars_x[frame], mars_y[frame])
        mars_point.set_3d_properties(mars_z[frame])

        spacecraft_point.set_data(sc_x[frame], sc_y[frame])
        spacecraft_point.set_3d_properties(sc_z[frame])

        return (earth_orbit, mars_orbit, earth_point, mars_point, spacecraft_point)

    ani = FuncAnimation(
        fig, update, frames=len(earth_x), init_func=init, blit=True, interval=50
    )

    plt.show()


if __name__ == "__main__":
    animate_launch()
