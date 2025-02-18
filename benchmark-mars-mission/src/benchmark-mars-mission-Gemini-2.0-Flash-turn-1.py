import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.constants import gravitational_constant as G

# Constants
EARTH_RADIUS = 6371e3  # meters
MARS_RADIUS = 3389.5e3  # meters
EARTH_GM = 3.986e14  # m^3/s^2  (Gravitational constant * Earth mass)
MARS_GM = 4.283e13  # m^3/s^2 (Gravitational constant * Mars mass)
EARTH_ORBITAL_RADIUS = 1.496e11  # meters
MARS_ORBITAL_RADIUS = 2.279e11  # meters
# Hohmann Transfer Orbit Parameters (Earth to Mars) - simplified
SEMI_MAJOR_AXIS_EM = (EARTH_ORBITAL_RADIUS + MARS_ORBITAL_RADIUS) / 2
TRANSFER_VELOCITY_EARTH = np.sqrt(EARTH_GM / EARTH_ORBITAL_RADIUS)  # circular velocity
TRANSFER_VELOCITY_MARS = np.sqrt(MARS_GM / MARS_ORBITAL_RADIUS)  # circular velocity

# Earth to Mars Hohmann Transfer parameters
V_EARTH_HO = np.sqrt(
    EARTH_GM
    * (2 / EARTH_ORBITAL_RADIUS - 2 / (EARTH_ORBITAL_RADIUS + MARS_ORBITAL_RADIUS))
)
V_MARS_HO = np.sqrt(
    MARS_GM
    * (2 / MARS_ORBITAL_RADIUS - 2 / (EARTH_ORBITAL_RADIUS + MARS_ORBITAL_RADIUS))
)

# Mars to Earth Hohmann Transfer Parameters
SEMI_MAJOR_AXIS_ME = (MARS_ORBITAL_RADIUS + EARTH_ORBITAL_RADIUS) / 2
V_MARS_HO_ME = np.sqrt(
    MARS_GM
    * (2 / MARS_ORBITAL_RADIUS - 2 / (MARS_ORBITAL_RADIUS + EARTH_ORBITAL_RADIUS))
)
V_EARTH_HO_ME = np.sqrt(
    EARTH_GM
    * (2 / EARTH_ORBITAL_RADIUS - 2 / (MARS_ORBITAL_RADIUS + EARTH_ORBITAL_RADIUS))
)

# Simplified launch windows calculation
SYNODIC_PERIOD = 780 * 24 * 3600  # seconds (Earth-Mars synodic period)
TRANSFER_TIME_EM = np.pi * np.sqrt(
    SEMI_MAJOR_AXIS_EM**3 / (G * 1.989e30)
)  # time to transfer to Mars
TRANSFER_TIME_ME = np.pi * np.sqrt(
    SEMI_MAJOR_AXIS_ME**3 / (G * 1.989e30)
)  # time to transfer to Earth


# Function to generate a circular orbit
def generate_circular_orbit(radius, num_points=100, inclination=0, ascending_node=0):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.zeros_like(x)

    # Apply inclination
    x_rot = x
    y_rot = y * np.cos(inclination) - z * np.sin(inclination)
    z_rot = y * np.sin(inclination) + z * np.cos(inclination)

    # Apply ascending node rotation
    x_final = x_rot * np.cos(ascending_node) - y_rot * np.sin(ascending_node)
    y_final = x_rot * np.sin(ascending_node) + y_rot * np.cos(ascending_node)
    z_final = z_rot

    return x_final, y_final, z_final


# Function to simulate Hohmann transfer orbit
def generate_hohmann_transfer_orbit(r_start, r_end, num_points=100):
    a = (r_start + r_end) / 2
    e = (r_end - r_start) / (r_end + r_start)
    theta = np.linspace(0, np.pi, num_points)
    r = a * (1 - e**2) / (1 + e * np.cos(theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.zeros_like(x)
    return x, y, z


# Set up the plot
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")
ax.set_aspect("equal")  # Ensure proper scaling
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")

# Scale down the plot so you can see the orbits
scale = 1e11  # Scale distances from meters to 10^11 meters (roughly AU)
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-2.5, 2.5])
ax.set_zlim([-2.5, 2.5])

# Create the planets and orbits
earth_x, earth_y, earth_z = generate_circular_orbit(EARTH_ORBITAL_RADIUS / scale)
mars_x, mars_y, mars_z = generate_circular_orbit(MARS_ORBITAL_RADIUS / scale)

(earth_orbit,) = ax.plot(earth_x, earth_y, earth_z, label="Earth Orbit", alpha=0.5)
(mars_orbit,) = ax.plot(mars_x, mars_y, mars_z, label="Mars Orbit", alpha=0.5)

earth = ax.plot(
    [earth_x[0]],
    [earth_y[0]],
    [earth_z[0]],
    marker="o",
    markersize=10,
    color="blue",
    label="Earth",
)[0]
mars = ax.plot(
    [mars_x[0]],
    [mars_y[0]],
    [mars_z[0]],
    marker="o",
    markersize=8,
    color="red",
    label="Mars",
)[0]
sun = ax.plot([0], [0], [0], marker="o", markersize=15, color="yellow", label="Sun")[0]

# Earth to Mars Hohmann Transfer Orbit
hohmann_x_em, hohmann_y_em, hohmann_z_em = generate_hohmann_transfer_orbit(
    EARTH_ORBITAL_RADIUS / scale, MARS_ORBITAL_RADIUS / scale
)
(hohmann_orbit_em,) = ax.plot(
    hohmann_x_em,
    hohmann_y_em,
    hohmann_z_em,
    color="green",
    label="Hohmann (Earth-Mars)",
    linestyle="--",
)

# Mars to Earth Hohmann Transfer Orbit
hohmann_x_me, hohmann_y_me, hohmann_z_me = generate_hohmann_transfer_orbit(
    MARS_ORBITAL_RADIUS / scale, EARTH_ORBITAL_RADIUS / scale
)
(hohmann_orbit_me,) = ax.plot(
    hohmann_x_me,
    hohmann_y_me,
    hohmann_z_me,
    color="purple",
    label="Hohmann (Mars-Earth)",
    linestyle="--",
)


# Spacecraft trajectory
num_transfer_points = 100
(spacecraft,) = ax.plot(
    [], [], [], marker=".", markersize=5, color="white", label="Spacecraft"
)
spacecraft_x_em = np.array([])  # Earth to Mars path
spacecraft_y_em = np.array([])
spacecraft_z_em = np.array([])
spacecraft_x_me = np.array([])  # Mars to Earth path
spacecraft_y_me = np.array([])
spacecraft_z_me = np.array([])


ax.legend()
ax.set_title("Earth-Mars-Earth Transfer Orbit")


def init():
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    return (spacecraft,)


def animate(frame):
    # Earth position
    earth_angle = (frame / 365.25) * 2 * np.pi  # Assume Earth's year is 365.25 days
    earth_x_pos = (EARTH_ORBITAL_RADIUS / scale) * np.cos(earth_angle)
    earth_y_pos = (EARTH_ORBITAL_RADIUS / scale) * np.sin(earth_angle)
    earth.set_data([earth_x_pos], [earth_y_pos])
    earth.set_3d_properties([0])

    # Mars position
    mars_angle = (
        (frame / (365.25 * 1.88)) * 2 * np.pi
    )  # Assume Mars's year is 1.88 Earth years
    mars_x_pos = (MARS_ORBITAL_RADIUS / scale) * np.cos(mars_angle)
    mars_y_pos = (MARS_ORBITAL_RADIUS / scale) * np.sin(mars_angle)
    mars.set_data([mars_x_pos], [mars_y_pos])
    mars.set_3d_properties([0])

    # Spacecraft Logic:  Simplified, using Hohmann transfer positions.
    # Needs refinement for realistic trajectory calculations.
    global spacecraft_x_em, spacecraft_y_em, spacecraft_z_em, spacecraft_x_me, spacecraft_y_me, spacecraft_z_me

    # Earth to Mars Transfer (Simplified)
    if frame < 180:  # Approximately half of 365, showing the transfer period
        index = int(
            (frame / 180) * (num_transfer_points - 1)
        )  # Scale frame to hohmann points
        spacecraft_x_em = np.append(spacecraft_x_em, hohmann_x_em[index])
        spacecraft_y_em = np.append(spacecraft_y_em, hohmann_y_em[index])
        spacecraft_z_em = np.append(spacecraft_z_em, hohmann_z_em[index])

        spacecraft.set_data(spacecraft_x_em, spacecraft_y_em)
        spacecraft.set_3d_properties(spacecraft_z_em)

    elif frame < 180 + 300:  # Wait 300 frames on Mars
        spacecraft.set_data([mars_x_pos], [mars_y_pos])  # park on mars
        spacecraft.set_3d_properties([0])

    elif frame < 180 + 300 + 180:  # Mars to Earth Transfer (Simplified)
        index = int(
            ((frame - (180 + 300)) / 180) * (num_transfer_points - 1)
        )  # Scale frame to hohmann points
        spacecraft_x_me = np.append(spacecraft_x_me, hohmann_x_me[index])
        spacecraft_y_me = np.append(spacecraft_y_me, hohmann_y_me[index])
        spacecraft_z_me = np.append(spacecraft_z_me, hohmann_z_me[index])

        spacecraft.set_data(spacecraft_x_me, spacecraft_y_me)
        spacecraft.set_3d_properties(spacecraft_z_me)

    else:
        spacecraft.set_data([earth_x_pos], [earth_y_pos])  # park on earth
        spacecraft.set_3d_properties([0])

    return spacecraft, earth, mars


# Create the animation
ani = FuncAnimation(fig, animate, init_func=init, frames=700, blit=False, repeat=True)

plt.show()
