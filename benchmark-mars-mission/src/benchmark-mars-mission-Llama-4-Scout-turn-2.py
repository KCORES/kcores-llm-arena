import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
G = 6.67430e-11  # Universal gravitational constant [m^3 kg^-1 s^-2]
M_earth = 5.97237e24  # Mass of Earth [kg]
M_mars = 6.4185e23  # Mass of Mars [kg]
R_earth = 6371000  # Radius of Earth [m]
R_mars = 3389500  # Radius of Mars [m]
earth_orbit_radius = 149.6e9  # Average distance from Sun to Earth [m]
mars_orbit_radius = 227.9e9  # Average distance from Sun to Mars [m]

# Initial conditions
v_escape_earth = np.sqrt(2 * G * M_earth / R_earth)  # Escape velocity from Earth [m/s]
v_orbit_earth = np.sqrt(G * M_earth / R_earth)  # Orbital velocity around Earth [m/s]
v_orbit_mars = np.sqrt(G * M_mars / R_mars)  # Orbital velocity around Mars [m/s]

# Trajectory parameters
theta_launch = np.radians(90)  # Launch angle from Earth's surface [rad]
theta_transfer = np.radians(120)  # Transfer orbit angle [rad]

# Time parameters
t_flight_earth_mars = 260 * 24 * 3600  # Flight time from Earth to Mars [s]
t_flight_mars_earth = 260 * 24 * 3600  # Flight time from Mars to Earth [s]

# Create figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


# Function to update plot
def update(frame):
    ax.clear()

    # Earth and Mars positions
    earth_pos = np.array([0, 0, 0])
    mars_pos = np.array(
        [
            mars_orbit_radius * np.cos(frame * np.pi / 180),
            mars_orbit_radius * np.sin(frame * np.pi / 180),
            0,
        ]
    )

    # Spacecraft position
    if frame < 180:
        # Launch from Earth
        r_sc = R_earth + frame * 10000  # Radius of spacecraft [m]
        theta_sc = frame * np.pi / 180  # Angle of spacecraft [rad]
        x_sc = r_sc * np.cos(theta_sc)
        y_sc = r_sc * np.sin(theta_sc)
        z_sc = 0
        ax.plot([earth_pos[0], x_sc], [earth_pos[1], y_sc], [earth_pos[2], z_sc], "b-")
    elif frame < 440:
        # Transfer orbit to Mars
        t_transfer = (frame - 180) * 24 * 3600  # Time in transfer orbit [s]
        r_sc = earth_orbit_radius + t_transfer * 1e-5  # Radius of spacecraft [m]
        theta_sc = theta_launch + t_transfer * 1e-5  # Angle of spacecraft [rad]
        x_sc = r_sc * np.cos(theta_sc)
        y_sc = r_sc * np.sin(theta_sc)
        z_sc = 0
        ax.plot([earth_pos[0], x_sc], [earth_pos[1], y_sc], [earth_pos[2], z_sc], "b-")
        ax.scatter(mars_pos[0], mars_pos[1], mars_pos[2], c="r")  # Mars position
    else:
        # Return journey
        t_return = (frame - 440) * 24 * 3600  # Time in return journey [s]
        r_sc = mars_orbit_radius - t_return * 1e-5  # Radius of spacecraft [m]
        theta_sc = np.pi - theta_transfer + t_return * 1e-5  # Angle of spacecraft [rad]
        x_sc = r_sc * np.cos(theta_sc)
        y_sc = r_sc * np.sin(theta_sc)
        z_sc = 0
        ax.plot([mars_pos[0], x_sc], [mars_pos[1], y_sc], [mars_pos[2], z_sc], "b-")

    # Plot Earth and Mars
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x_earth = R_earth * np.outer(np.cos(u), np.sin(v))
    y_earth = R_earth * np.outer(np.sin(u), np.sin(v))
    z_earth = R_earth * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x_earth, y_earth, z_earth, color="green", alpha=0.3)

    x_mars = R_mars * np.outer(np.cos(u), np.sin(v)) + mars_pos[0]
    y_mars = R_mars * np.outer(np.sin(u), np.sin(v)) + mars_pos[1]
    z_mars = R_mars * np.outer(np.ones(np.size(u)), np.cos(v)) + mars_pos[2]
    ax.plot_surface(x_mars, y_mars, z_mars, color="red", alpha=0.3)

    ax.set_xlim(-2e11, 2e11)
    ax.set_ylim(-2e11, 2e11)
    ax.set_zlim(-2e11, 2e11)


ani = animation.FuncAnimation(fig, update, frames=range(0, 720), interval=50)

plt.show()
