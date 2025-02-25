import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as colors

# Constants
AU = 149.6e6  # Astronomical Unit in km
EARTH_RADIUS = 6371  # km (not to scale in visualization)
MARS_RADIUS = 3389.5  # km (not to scale in visualization)
SUN_RADIUS = 696340  # km (not to scale in visualization)

# Orbital parameters
EARTH_SEMI_MAJOR_AXIS = 1.0 * AU
EARTH_ECCENTRICITY = 0.0167
EARTH_ORBITAL_PERIOD = 365.25  # days
EARTH_INCLINATION = np.radians(0.0)  # Earth's orbit defines the reference plane

MARS_SEMI_MAJOR_AXIS = 1.524 * AU
MARS_ECCENTRICITY = 0.0934
MARS_ORBITAL_PERIOD = 687  # days
MARS_INCLINATION = np.radians(1.85)  # Mars' orbit is slightly inclined

# Mission parameters
EARTH_TO_MARS_TRANSFER_TIME = 259  # days
STAY_ON_MARS = 500  # days (waiting for next launch window)
MARS_TO_EARTH_TRANSFER_TIME = 239  # days

TOTAL_MISSION_TIME = (
    EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS + MARS_TO_EARTH_TRANSFER_TIME
)
NUM_FRAMES = 360  # Total frames for animation

# Create figure and 3D axis
plt.style.use("dark_background")  # Space is dark!
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")


# Function to calculate position in elliptical orbit with inclination
def get_orbit_position(semi_major_axis, eccentricity, inclination, true_anomaly):
    # Calculate radius from focus
    r = (
        semi_major_axis
        * (1 - eccentricity**2)
        / (1 + eccentricity * np.cos(true_anomaly))
    )

    # Convert to Cartesian coordinates
    x = r * np.cos(true_anomaly)
    y = r * np.sin(true_anomaly) * np.cos(inclination)
    z = r * np.sin(true_anomaly) * np.sin(inclination)

    return x, y, z


# Function to calculate planet positions at given time
def calculate_planet_positions(t_days):
    # Calculate true anomaly for each planet
    earth_anomaly = 2 * np.pi * t_days / EARTH_ORBITAL_PERIOD
    mars_anomaly = (
        2 * np.pi * t_days / MARS_ORBITAL_PERIOD + 0.5
    )  # Phase offset for Mars

    # Get positions
    earth_x, earth_y, earth_z = get_orbit_position(
        EARTH_SEMI_MAJOR_AXIS, EARTH_ECCENTRICITY, EARTH_INCLINATION, earth_anomaly
    )
    mars_x, mars_y, mars_z = get_orbit_position(
        MARS_SEMI_MAJOR_AXIS, MARS_ECCENTRICITY, MARS_INCLINATION, mars_anomaly
    )

    return earth_x, earth_y, earth_z, mars_x, mars_y, mars_z


# Calculate transfer orbit parameters
def calculate_transfer_orbit(
    r1,
    r2,
    departure_anomaly,
    arrival_anomaly,
    progress,
    inclination_start,
    inclination_end,
):
    # Semi-major axis of transfer orbit
    a_transfer = (r1 + r2) / 2

    # Eccentricity
    e_transfer = abs(r2 - r1) / (r2 + r1)

    # Current anomaly based on interpolation (this is a simplification)
    current_anomaly = departure_anomaly + progress * (
        arrival_anomaly - departure_anomaly
    )

    # Interpolate inclination
    current_inclination = inclination_start + progress * (
        inclination_end - inclination_start
    )

    # Get position on transfer orbit
    x, y, z = get_orbit_position(
        a_transfer, e_transfer, current_inclination, current_anomaly
    )

    # Add additional z-component for visualization
    z_boost = 0.05 * AU * np.sin(np.pi * progress)
    z += z_boost

    return x, y, z


# Function to calculate spacecraft position
def calculate_spacecraft_position(t_days):
    if t_days < EARTH_TO_MARS_TRANSFER_TIME:
        # Earth to Mars transfer
        progress = t_days / EARTH_TO_MARS_TRANSFER_TIME

        # Get positions at departure and arrival
        earth_x_dep, earth_y_dep, earth_z_dep, _, _, _ = calculate_planet_positions(0)
        _, _, _, mars_x_arr, mars_y_arr, mars_z_arr = calculate_planet_positions(
            EARTH_TO_MARS_TRANSFER_TIME
        )

        r1 = np.sqrt(earth_x_dep**2 + earth_y_dep**2 + earth_z_dep**2)
        r2 = np.sqrt(mars_x_arr**2 + mars_y_arr**2 + mars_z_arr**2)

        departure_anomaly = 0
        arrival_anomaly = np.pi

        x, y, z = calculate_transfer_orbit(
            r1,
            r2,
            departure_anomaly,
            arrival_anomaly,
            progress,
            EARTH_INCLINATION,
            MARS_INCLINATION,
        )

    elif t_days < EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS:
        # On Mars
        mars_time = t_days
        _, _, _, x, y, z = calculate_planet_positions(mars_time)

    else:
        # Mars to Earth transfer
        return_time = t_days - (EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS)
        progress = return_time / MARS_TO_EARTH_TRANSFER_TIME

        # Get positions at departure and arrival
        _, _, _, mars_x_dep, mars_y_dep, mars_z_dep = calculate_planet_positions(
            EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS
        )
        earth_x_arr, earth_y_arr, earth_z_arr, _, _, _ = calculate_planet_positions(
            TOTAL_MISSION_TIME
        )

        r1 = np.sqrt(mars_x_dep**2 + mars_y_dep**2 + mars_z_dep**2)
        r2 = np.sqrt(earth_x_arr**2 + earth_y_arr**2 + earth_z_arr**2)

        departure_anomaly = np.pi
        arrival_anomaly = 2 * np.pi

        x, y, z = calculate_transfer_orbit(
            r1,
            r2,
            departure_anomaly,
            arrival_anomaly,
            progress,
            MARS_INCLINATION,
            EARTH_INCLINATION,
        )

    return x, y, z


# Calculate and store orbit paths
def calculate_orbit_paths():
    theta = np.linspace(0, 2 * np.pi, 100)

    earth_x, earth_y, earth_z = [], [], []
    mars_x, mars_y, mars_z = [], [], []

    for angle in theta:
        ex, ey, ez = get_orbit_position(
            EARTH_SEMI_MAJOR_AXIS, EARTH_ECCENTRICITY, EARTH_INCLINATION, angle
        )
        mx, my, mz = get_orbit_position(
            MARS_SEMI_MAJOR_AXIS, MARS_ECCENTRICITY, MARS_INCLINATION, angle
        )

        earth_x.append(ex)
        earth_y.append(ey)
        earth_z.append(ez)

        mars_x.append(mx)
        mars_y.append(my)
        mars_z.append(mz)

    return earth_x, earth_y, earth_z, mars_x, mars_y, mars_z


# Calculate orbit paths
(
    earth_orbit_x,
    earth_orbit_y,
    earth_orbit_z,
    mars_orbit_x,
    mars_orbit_y,
    mars_orbit_z,
) = calculate_orbit_paths()

# Spacecraft trail storage
spacecraft_trail_x, spacecraft_trail_y, spacecraft_trail_z = [], [], []


# Draw a sphere for celestial bodies
def plot_sphere(ax, x, y, z, radius, color):
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 20)

    x_surface = x + radius * np.outer(np.cos(u), np.sin(v))
    y_surface = y + radius * np.outer(np.sin(u), np.sin(v))
    z_surface = z + radius * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x_surface, y_surface, z_surface, color=color, alpha=0.9)


# Initialize the plot
def init():
    ax.clear()

    # Plot Sun (scaled for visualization)
    sun_vis_radius = 0.1 * AU
    plot_sphere(ax, 0, 0, 0, sun_vis_radius, "yellow")

    # Plot orbit paths
    ax.plot(earth_orbit_x, earth_orbit_y, earth_orbit_z, color="blue", alpha=0.3)
    ax.plot(mars_orbit_x, mars_orbit_y, mars_orbit_z, color="red", alpha=0.3)

    # Set labels
    ax.set_xlabel("X (km)")
    ax.set_ylabel("Y (km)")
    ax.set_zlabel("Z (km)")

    # Set limits
    limit = 1.8 * AU
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-0.5 * AU, 0.5 * AU)

    # Remove axis grid and ticks for cleaner look
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    # Title
    ax.set_title("Earth to Mars Round Trip Mission", fontsize=16, color="white")

    return []


# Update function for animation
def update(frame):
    # Calculate time in days
    t_days = frame * TOTAL_MISSION_TIME / NUM_FRAMES

    # Clear previous plot elements
    init()

    # Get positions
    earth_x, earth_y, earth_z, mars_x, mars_y, mars_z = calculate_planet_positions(
        t_days
    )
    spacecraft_x, spacecraft_y, spacecraft_z = calculate_spacecraft_position(t_days)

    # Plot Earth and Mars (scaled for visualization)
    earth_vis_radius = 0.03 * AU
    mars_vis_radius = 0.02 * AU

    plot_sphere(ax, earth_x, earth_y, earth_z, earth_vis_radius, "blue")
    plot_sphere(ax, mars_x, mars_y, mars_z, mars_vis_radius, "red")

    # Plot spacecraft
    spacecraft = ax.scatter(
        [spacecraft_x],
        [spacecraft_y],
        [spacecraft_z],
        color="white",
        s=30,
        edgecolor="cyan",
    )

    # Update trail
    spacecraft_trail_x.append(spacecraft_x)
    spacecraft_trail_y.append(spacecraft_y)
    spacecraft_trail_z.append(spacecraft_z)

    # Limit trail length
    trail_length = 150
    if len(spacecraft_trail_x) > trail_length:
        spacecraft_trail_x.pop(0)
        spacecraft_trail_y.pop(0)
        spacecraft_trail_z.pop(0)

    # Plot trail with gradient color
    if len(spacecraft_trail_x) > 1:
        segments_x = spacecraft_trail_x
        segments_y = spacecraft_trail_y
        segments_z = spacecraft_trail_z

        # Create gradient colors
        colors_array = plt.cm.cool(np.linspace(0, 1, len(segments_x)))

        # Plot trail segments with gradient colors
        for i in range(len(segments_x) - 1):
            ax.plot(
                segments_x[i : i + 2],
                segments_y[i : i + 2],
                segments_z[i : i + 2],
                color=colors_array[i],
                alpha=(i / len(segments_x)) ** 1.5 + 0.3,
                linewidth=2,
            )

    # Add mission phase text
    if t_days < EARTH_TO_MARS_TRANSFER_TIME:
        progress = int(100 * t_days / EARTH_TO_MARS_TRANSFER_TIME)
        phase_txt = f"Earth to Mars Transfer: Day {int(t_days)} / {int(EARTH_TO_MARS_TRANSFER_TIME)} [{progress}%]"
    elif t_days < EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS:
        mars_day = t_days - EARTH_TO_MARS_TRANSFER_TIME
        progress = int(100 * mars_day / STAY_ON_MARS)
        phase_txt = f"Mars Surface Operations: Day {int(mars_day)} / {int(STAY_ON_MARS)} [{progress}%]"
    else:
        return_day = t_days - (EARTH_TO_MARS_TRANSFER_TIME + STAY_ON_MARS)
        progress = int(100 * return_day / MARS_TO_EARTH_TRANSFER_TIME)
        phase_txt = f"Mars to Earth Return: Day {int(return_day)} / {int(MARS_TO_EARTH_TRANSFER_TIME)} [{progress}%]"

    # Add text for phase information
    plt.figtext(0.5, 0.02, phase_txt, ha="center", fontsize=14, color="white")

    # Add mission day counter
    day_txt = f"Mission Day: {int(t_days)} / {int(TOTAL_MISSION_TIME)}"
    plt.figtext(0.5, 0.06, day_txt, ha="center", fontsize=12, color="white")

    # Rotate view for more dynamic visualization
    azimuth = frame % 360
    elevation = 20 + 10 * np.sin(frame * 0.02)
    ax.view_init(elev=elevation, azim=azimuth)

    return [spacecraft]


# Create animation
anim = FuncAnimation(
    fig, update, frames=NUM_FRAMES, init_func=init, interval=50, blit=False
)

# Save animation as GIF or MP4
# MP4 requires ffmpeg to be installed
anim.save("mars_mission_trajectory.mp4", writer="ffmpeg", fps=30, dpi=100)
# For GIF alternative:
# anim.save('mars_mission_trajectory.gif', writer='pillow', fps=30, dpi=80)

plt.tight_layout()
plt.show()
