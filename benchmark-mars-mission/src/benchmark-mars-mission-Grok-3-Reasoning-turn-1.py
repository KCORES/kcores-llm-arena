import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
AU = 1.0  # Astronomical Unit in arbitrary units
MARS_AU = 1.524  # Mars semi-major axis in AU
EARTH_PERIOD = 1.0  # Earth orbital period in years
MARS_PERIOD = 1.88  # Mars orbital period in years
TRANSFER_TIME = 0.709  # Hohmann transfer time in years (approx 259 days)
STAY_ON_MARS = 500 / 365.25  # Stay duration on Mars in years (approx 500 days)
TOTAL_DURATION = (TRANSFER_TIME * 2 + STAY_ON_MARS) * 365.25  # Total mission in days


# Convert days to years for calculations
def days_to_years(days):
    return days / 365.25


# Planetary positions (circular orbits, xy-plane)
def get_earth_position(t):
    t_years = days_to_years(t)
    theta = 2 * np.pi * t_years / EARTH_PERIOD
    return AU * np.cos(theta), AU * np.sin(theta), 0


def get_mars_position(t):
    t_years = days_to_years(t)
    theta = 2 * np.pi * t_years / MARS_PERIOD
    return MARS_AU * np.cos(theta), MARS_AU * np.sin(theta), 0


# Hohmann transfer orbit parameters
a_transfer = (AU + MARS_AU) / 2  # Semi-major axis of transfer orbit
e_transfer = (MARS_AU - AU) / (AU + MARS_AU)  # Eccentricity
T_transfer = 2 * TRANSFER_TIME  # Period of transfer orbit (full orbit)


def solve_keplers_equation(M, e, tol=1e-8, max_iter=100):
    """Solve Kepler's equation M = E - e sin E for eccentric anomaly E."""
    E = M if M < np.pi else M - e  # Initial guess
    for _ in range(max_iter):
        delta = (E - e * np.sin(E) - M) / (1 - e * np.cos(E))
        E -= delta
        if abs(delta) < tol:
            break
    return E


def get_transfer_position(t, is_return=False, launch_time=0):
    """Calculate spacecraft position during transfer orbit."""
    t_years = days_to_years(t - launch_time)
    if t_years < 0 or t_years > TRANSFER_TIME:
        return np.nan, np.nan, np.nan

    # Mean motion
    n = 2 * np.pi / T_transfer
    M0 = np.pi if is_return else 0  # Start at apogee for return, perigee for outbound
    M = (
        M0 + n * t_years if not is_return else M0 + (np.pi / TRANSFER_TIME) * t_years
    )  # Adjust for return

    # Solve for eccentric anomaly
    E = solve_keplers_equation(M % (2 * np.pi), e_transfer)

    # Position in transfer orbit coordinate system (sun at focus)
    x = a_transfer * (np.cos(E) - e_transfer)
    y = a_transfer * np.sqrt(1 - e_transfer**2) * np.sin(E)
    z = 0

    # Rotate to align with launch position (simplified, assume launch at Earth position)
    launch_pos = (
        get_earth_position(launch_time)
        if not is_return
        else get_mars_position(launch_time)
    )
    theta = np.arctan2(launch_pos[1], launch_pos[0])
    x_rot = x * np.cos(theta) - y * np.sin(theta)
    y_rot = x * np.sin(theta) + y * np.cos(theta)

    return x_rot, y_rot, z


# Animation setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1.6, 1.6)
ax.set_ylim(-1.6, 1.6)
ax.set_zlim(-0.1, 0.1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Spacecraft Journey: Earth to Mars and Back")

# Plot sun
ax.scatter([0], [0], [0], color="yellow", s=100, label="Sun")

# Initialize lists for plotting
(earth_line,) = ax.plot([], [], [], "bo-", label="Earth")
(mars_line,) = ax.plot([], [], [], "ro-", label="Mars")
(spacecraft_line,) = ax.plot([], [], [], "g.-", label="Spacecraft")

ax.legend()


# Animation function
def animate(i):
    # Clear previous positions
    earth_line.set_data([], [])
    earth_line.set_3d_properties([])
    mars_line.set_data([], [])
    mars_line.set_3d_properties([])
    spacecraft_line.set_data([], [])
    spacecraft_line.set_3d_properties([])

    # Get positions
    t = i  # Time in days
    x_e, y_e, z_e = get_earth_position(t)
    x_m, y_m, z_m = get_mars_position(t)

    # Plot Earth and Mars orbits (full circles for reference)
    earth_theta = np.linspace(0, 2 * np.pi, 100)
    earth_x = AU * np.cos(earth_theta)
    earth_y = AU * np.sin(earth_theta)
    mars_x = MARS_AU * np.cos(earth_theta)
    mars_y = MARS_AU * np.sin(earth_theta)
    ax.plot(earth_x, earth_y, np.zeros_like(earth_x), "b-", alpha=0.3)
    ax.plot(mars_x, mars_y, np.zeros_like(mars_x), "r-", alpha=0.3)

    # Plot Earth and Mars current positions
    earth_line.set_data([x_e], [y_e])
    earth_line.set_3d_properties([z_e])
    mars_line.set_data([x_m], [y_m])
    mars_line.set_3d_properties([z_m])

    # Spacecraft position
    if t <= TRANSFER_TIME * 365.25:  # Outbound to Mars
        x_s, y_s, z_s = get_transfer_position(t, False, 0)
    elif t <= (TRANSFER_TIME + STAY_ON_MARS) * 365.25:  # On Mars
        x_s, y_s, z_s = get_mars_position(t)
    elif t <= TOTAL_DURATION:  # Return to Earth
        launch_mars_time = (TRANSFER_TIME + STAY_ON_MARS) * 365.25
        x_s, y_s, z_s = get_transfer_position(t, True, launch_mars_time)
    else:
        x_s, y_s, z_s = np.nan, np.nan, np.nan

    spacecraft_line.set_data([x_s], [y_s])
    spacecraft_line.set_3d_properties([z_s])

    return earth_line, mars_line, spacecraft_line


# Create animation
ani = animation.FuncAnimation(
    fig, animate, frames=int(TOTAL_DURATION) + 1, interval=50, blit=True
)

# Save or show animation
plt.show()
# To save: ani.save('spacecraft_journey.mp4', writer='ffmpeg', dpi=200)
