import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


def create_space_animation():
    # Set up the figure and 3D axes
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Parameters for Earth and Mars orbits
    earth_radius = 1.0  # AU (Astronomical Unit)
    mars_radius = 1.524  # AU

    # Create orbit paths
    theta = np.linspace(0, 2 * np.pi, 100)

    # Earth orbit
    earth_x = earth_radius * np.cos(theta)
    earth_y = earth_radius * np.sin(theta)
    earth_z = np.zeros_like(theta)

    # Mars orbit
    mars_x = mars_radius * np.cos(theta)
    mars_y = mars_radius * np.sin(theta)
    mars_z = np.zeros_like(theta)

    # Spacecraft trajectory (simplified Hohmann transfer)
    def spacecraft_position(t):
        if t < 0.5:  # Earth to Mars
            r = earth_radius + (mars_radius - earth_radius) * (2 * t)
            angle = np.pi * t
        else:  # Mars to Earth
            r = mars_radius - (mars_radius - earth_radius) * (2 * (t - 0.5))
            angle = np.pi + np.pi * (t - 0.5)

        x = r * np.cos(angle)
        y = r * np.sin(angle)
        z = 0.1 * np.sin(2 * np.pi * t)  # Add some vertical movement
        return x, y, z

    def animate(frame):
        ax.clear()

        # Plot orbits
        ax.plot(earth_x, earth_y, earth_z, "b--", alpha=0.3, label="Earth Orbit")
        ax.plot(mars_x, mars_y, mars_z, "r--", alpha=0.3, label="Mars Orbit")

        # Plot planets
        earth_pos = (
            earth_radius * np.cos(frame * 2 * np.pi),
            earth_radius * np.sin(frame * 2 * np.pi),
            0,
        )
        mars_pos = (
            mars_radius * np.cos(frame * 2 * np.pi / 1.88),
            mars_radius * np.sin(frame * 2 * np.pi / 1.88),
            0,
        )

        ax.scatter(*earth_pos, color="blue", s=100, label="Earth")
        ax.scatter(*mars_pos, color="red", s=80, label="Mars")

        # Plot spacecraft
        sc_pos = spacecraft_position(frame)
        ax.scatter(*sc_pos, color="white", s=50, edgecolor="green", label="Spacecraft")

        # Customize the plot
        ax.set_xlabel("X (AU)")
        ax.set_ylabel("Y (AU)")
        ax.set_zlabel("Z (AU)")
        ax.set_title("Earth-Mars-Earth Transfer Trajectory")
        ax.legend()

        # Set consistent view limits
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_zlim(-0.5, 0.5)

        # Set view angle
        ax.view_init(elev=20, azim=frame * 360)

    # Create animation
    anim = FuncAnimation(fig, animate, frames=np.linspace(0, 1, 200), interval=50)

    return anim


# Create and display animation
anim = create_space_animation()
plt.show()
