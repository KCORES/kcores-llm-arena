import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
earth_radius = 6371  # km
mars_radius = 3390  # km
earth_mars_distance = 56000000  # km (average, simplified)
earth_orbit_radius = 149.6e6  # km
mars_orbit_radius = 227.9e6  # km


# Function to compute position of planets and spacecraft
def compute_positions(t):
    earth_x = earth_orbit_radius * np.cos(t)
    earth_y = earth_orbit_radius * np.sin(t)
    earth_z = 0

    mars_x = mars_orbit_radius * np.cos(t * 0.5)  # Mars has a different orbit period
    mars_y = mars_orbit_radius * np.sin(t * 0.5)
    mars_z = 0

    # Simple Hohmann transfer orbit approximation
    if t < np.pi:  # From Earth to Mars
        theta = t
        r = (earth_orbit_radius + mars_orbit_radius) / 2
        spacecraft_x = r * np.cos(theta)
        spacecraft_y = r * np.sin(theta)
        spacecraft_z = 0
    else:  # From Mars back to Earth
        theta = t - np.pi
        r = (earth_orbit_radius + mars_orbit_radius) / 2
        spacecraft_x = mars_x + r * np.cos(theta)
        spacecraft_y = mars_y + r * np.sin(theta)
        spacecraft_z = 0

    return (
        (earth_x, earth_y, earth_z),
        (mars_x, mars_y, mars_z),
        (spacecraft_x, spacecraft_y, spacecraft_z),
    )


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Set up the plot
ax.set_xlim(-300e6, 300e6)
ax.set_ylim(-300e6, 300e6)
ax.set_zlim(-50e6, 50e6)
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")

(earth_plot,) = ax.plot([], [], [], "bo", markersize=10)  # Blue dot for Earth
(mars_plot,) = ax.plot([], [], [], "ro", markersize=8)  # Red dot for Mars
(spacecraft_plot,) = ax.plot([], [], [], "go", markersize=5)  # Green dot for spacecraft


def init():
    earth_plot.set_data([], [])
    earth_plot.set_3d_properties([])
    mars_plot.set_data([], [])
    mars_plot.set_3d_properties([])
    spacecraft_plot.set_data([], [])
    spacecraft_plot.set_3d_properties([])
    return earth_plot, mars_plot, spacecraft_plot


def animate(i):
    t = i / 100.0 * 2 * np.pi  # Animate over one full orbit and back
    earth_pos, mars_pos, spacecraft_pos = compute_positions(t)

    earth_plot.set_data(earth_pos[0], earth_pos[1])
    earth_plot.set_3d_properties(earth_pos[2])
    mars_plot.set_data(mars_pos[0], mars_pos[1])
    mars_plot.set_3d_properties(mars_pos[2])
    spacecraft_plot.set_data(spacecraft_pos[0], spacecraft_pos[1])
    spacecraft_plot.set_3d_properties(spacecraft_pos[2])

    return earth_plot, mars_plot, spacecraft_plot


# Create animation
anim = FuncAnimation(fig, animate, init_func=init, frames=200, interval=50, blit=True)

plt.show()
