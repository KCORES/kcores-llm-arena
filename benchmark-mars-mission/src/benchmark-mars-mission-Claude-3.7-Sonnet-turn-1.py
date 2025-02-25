import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# Constants
AU = 149.6e6  # Astronomical Unit in km
EARTH_RADIUS = 6371  # km
MARS_RADIUS = 3389.5  # km
SUN_RADIUS = 695700  # km (not to scale in visualization)

# Orbital parameters
EARTH_ORBIT_RADIUS = 1.0 * AU  # 1 AU
MARS_ORBIT_RADIUS = 1.524 * AU  # 1.524 AU
EARTH_ORBITAL_PERIOD = 365.25  # days
MARS_ORBITAL_PERIOD = 687  # days

# Mission parameters
EARTH_TO_MARS_DURATION = 210  # days (typical Hohmann transfer)
STAY_ON_MARS = 500  # days (waiting for next launch window)
MARS_TO_EARTH_DURATION = 210  # days

# Total mission duration
TOTAL_MISSION_DURATION = EARTH_TO_MARS_DURATION + STAY_ON_MARS + MARS_TO_EARTH_DURATION


def calculate_planet_position(t, radius, period):
    """Calculate planet position at time t"""
    angle = 2 * np.pi * t / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0  # Assuming planets orbit in the same plane for simplicity
    return x, y, z


def calculate_spacecraft_position(t):
    """Calculate spacecraft position at time t"""
    if t < EARTH_TO_MARS_DURATION:
        # Earth to Mars transfer
        progress = t / EARTH_TO_MARS_DURATION
        # Get Earth position at launch
        earth_x0, earth_y0, earth_z0 = calculate_planet_position(
            0, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD
        )
        # Get Mars position at arrival
        mars_x1, mars_y1, mars_z1 = calculate_planet_position(
            EARTH_TO_MARS_DURATION, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )

        # Simple interpolation with some "arc" added
        x = earth_x0 + (mars_x1 - earth_x0) * progress
        y = earth_y0 + (mars_y1 - earth_y0) * progress
        # Add an arc to the trajectory
        z = np.sin(np.pi * progress) * 0.2 * AU

    elif t < EARTH_TO_MARS_DURATION + STAY_ON_MARS:
        # Staying on Mars
        mars_time = t
        x, y, z = calculate_planet_position(
            mars_time, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )

    else:
        # Mars to Earth transfer
        mars_departure_time = EARTH_TO_MARS_DURATION + STAY_ON_MARS
        progress = (t - mars_departure_time) / MARS_TO_EARTH_DURATION

        # Get Mars position at departure
        mars_x0, mars_y0, mars_z0 = calculate_planet_position(
            mars_departure_time, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )

        # Get Earth position at arrival
        earth_x1, earth_y1, earth_z1 = calculate_planet_position(
            TOTAL_MISSION_DURATION, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD
        )

        # Simple interpolation with some "arc" added
        x = mars_x0 + (earth_x1 - mars_x0) * progress
        y = mars_y0 + (earth_y1 - mars_y0) * progress
        # Add an arc to the trajectory
        z = np.sin(np.pi * progress) * 0.2 * AU

    return x, y, z


# Create figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")

# Create the Sun
theta = np.linspace(0, 2 * np.pi, 100)
phi = np.linspace(0, np.pi, 100)
x_sun = (
    SUN_RADIUS * np.outer(np.cos(theta), np.sin(phi)) * 0.02
)  # Scaled down for visibility
y_sun = SUN_RADIUS * np.outer(np.sin(theta), np.sin(phi)) * 0.02
z_sun = SUN_RADIUS * np.outer(np.ones(100), np.cos(phi)) * 0.02
sun = ax.plot_surface(x_sun, y_sun, z_sun, color="yellow", alpha=0.8)

# Draw Earth's orbit
theta_earth = np.linspace(0, 2 * np.pi, 100)
x_earth_orbit = EARTH_ORBIT_RADIUS * np.cos(theta_earth)
y_earth_orbit = EARTH_ORBIT_RADIUS * np.sin(theta_earth)
z_earth_orbit = np.zeros_like(theta_earth)
ax.plot(x_earth_orbit, y_earth_orbit, z_earth_orbit, "b-", alpha=0.3)

# Draw Mars' orbit
theta_mars = np.linspace(0, 2 * np.pi, 100)
x_mars_orbit = MARS_ORBIT_RADIUS * np.cos(theta_mars)
y_mars_orbit = MARS_ORBIT_RADIUS * np.sin(theta_mars)
z_mars_orbit = np.zeros_like(theta_mars)
ax.plot(x_mars_orbit, y_mars_orbit, z_mars_orbit, "r-", alpha=0.3)

# Initialize Earth, Mars and spacecraft
(earth_point,) = ax.plot([], [], [], "bo", markersize=10, label="Earth")
(mars_point,) = ax.plot([], [], [], "ro", markersize=8, label="Mars")
(spacecraft,) = ax.plot([], [], [], "ko", markersize=5, label="Spacecraft")

# Initialize trajectory lines
(earth_to_mars_trajectory,) = ax.plot([], [], [], "k-", alpha=0.5)
(mars_to_earth_trajectory,) = ax.plot([], [], [], "k-", alpha=0.5)

# Set axis limits
max_radius = 1.7 * AU
ax.set_xlim(-max_radius, max_radius)
ax.set_ylim(-max_radius, max_radius)
ax.set_zlim(-0.5 * AU, 0.5 * AU)

# Labels
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Mission Trajectory")

# Mission phase text
mission_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

# Legend
ax.legend()

# Store trajectory points
earth_to_mars_x, earth_to_mars_y, earth_to_mars_z = [], [], []
mars_to_earth_x, mars_to_earth_y, mars_to_earth_z = [], [], []


def init():
    earth_point.set_data([], [])
    earth_point.set_3d_properties([])
    mars_point.set_data([], [])
    mars_point.set_3d_properties([])
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    earth_to_mars_trajectory.set_data([], [])
    earth_to_mars_trajectory.set_3d_properties([])
    mars_to_earth_trajectory.set_data([], [])
    mars_to_earth_trajectory.set_3d_properties([])
    mission_text.set_text("")
    return (
        earth_point,
        mars_point,
        spacecraft,
        earth_to_mars_trajectory,
        mars_to_earth_trajectory,
        mission_text,
    )


def update(frame):
    t = frame * TOTAL_MISSION_DURATION / 200  # 200 frames for the entire mission

    # Update Earth position
    earth_x, earth_y, earth_z = calculate_planet_position(
        t, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD
    )
    earth_point.set_data([earth_x], [earth_y])
    earth_point.set_3d_properties([earth_z])

    # Update Mars position
    mars_x, mars_y, mars_z = calculate_planet_position(
        t, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
    )
    mars_point.set_data([mars_x], [mars_y])
    mars_point.set_3d_properties([mars_z])

    # Update spacecraft position
    sc_x, sc_y, sc_z = calculate_spacecraft_position(t)
    spacecraft.set_data([sc_x], [sc_y])
    spacecraft.set_3d_properties([sc_z])

    # Update trajectory
    if t < EARTH_TO_MARS_DURATION:
        earth_to_mars_x.append(sc_x)
        earth_to_mars_y.append(sc_y)
        earth_to_mars_z.append(sc_z)
        earth_to_mars_trajectory.set_data(earth_to_mars_x, earth_to_mars_y)
        earth_to_mars_trajectory.set_3d_properties(earth_to_mars_z)
        mission_text.set_text(f"Mission Phase: Earth to Mars Transfer (Day {t:.0f})")
    elif t < EARTH_TO_MARS_DURATION + STAY_ON_MARS:
        mission_text.set_text(f"Mission Phase: Mars Surface Operations (Day {t:.0f})")
    else:
        mars_to_earth_x.append(sc_x)
        mars_to_earth_y.append(sc_y)
        mars_to_earth_z.append(sc_z)
        mars_to_earth_trajectory.set_data(mars_to_earth_x, mars_to_earth_y)
        mars_to_earth_trajectory.set_3d_properties(mars_to_earth_z)
        mission_text.set_text(f"Mission Phase: Mars to Earth Transfer (Day {t:.0f})")

    # Adjust view angle for better perspective
    ax.view_init(elev=20, azim=frame / 2)

    return (
        earth_point,
        mars_point,
        spacecraft,
        earth_to_mars_trajectory,
        mars_to_earth_trajectory,
        mission_text,
    )


# Create animation
ani = FuncAnimation(fig, update, frames=200, init_func=init, blit=False, interval=100)

plt.tight_layout()
plt.show()

# Uncomment to save animation
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)
