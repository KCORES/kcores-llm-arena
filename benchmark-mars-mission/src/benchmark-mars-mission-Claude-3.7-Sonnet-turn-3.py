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

# Orbital parameters (simplified)
EARTH_ORBIT_RADIUS = 1.0 * AU  # 1 AU
MARS_ORBIT_RADIUS = 1.524 * AU  # 1.524 AU
EARTH_ORBITAL_PERIOD = 365.25  # days
MARS_ORBITAL_PERIOD = 687  # days

# Mission timeline (in days)
EARTH_TO_MARS_DURATION = 210  # Typical transit time
STAY_ON_MARS = 500  # Wait for next launch window
MARS_TO_EARTH_DURATION = 210  # Return journey

TOTAL_MISSION_DURATION = EARTH_TO_MARS_DURATION + STAY_ON_MARS + MARS_TO_EARTH_DURATION
FRAMES = 500  # Total animation frames


def create_sphere(radius, resolution=20):
    """Create a sphere with given radius and resolution"""
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z


def planet_position(t, orbit_radius, orbital_period):
    """Calculate planet position at time t"""
    angle = 2 * np.pi * t / orbital_period
    x = orbit_radius * np.cos(angle)
    y = orbit_radius * np.sin(angle)
    z = 0  # Simplified 2D orbits in 3D space
    return x, y, z


def spacecraft_trajectory(t, mission_phase):
    """Calculate spacecraft position at time t based on mission phase"""
    if mission_phase == "earth_to_mars":
        # Progress from 0 to 1 during Earth to Mars journey
        progress = t / EARTH_TO_MARS_DURATION

        # Get positions of Earth and Mars at departure and arrival times
        earth_pos_departure = planet_position(
            0, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD
        )
        mars_pos_arrival = planet_position(
            EARTH_TO_MARS_DURATION, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )

        # Simple interpolation with a slight curve
        x = earth_pos_departure[0] + progress * (
            mars_pos_arrival[0] - earth_pos_departure[0]
        )
        y = earth_pos_departure[1] + progress * (
            mars_pos_arrival[1] - earth_pos_departure[1]
        )
        # Add a slight arc to the trajectory
        z = np.sin(np.pi * progress) * 0.2 * AU

    elif mission_phase == "mars_stay":
        # Stay with Mars
        return planet_position(
            EARTH_TO_MARS_DURATION + t, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )

    elif mission_phase == "mars_to_earth":
        # Progress from 0 to 1 during Mars to Earth journey
        progress = t / MARS_TO_EARTH_DURATION

        # Get positions of Mars and Earth at departure and arrival times
        departure_time = EARTH_TO_MARS_DURATION + STAY_ON_MARS
        arrival_time = TOTAL_MISSION_DURATION

        mars_pos_departure = planet_position(
            departure_time, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD
        )
        earth_pos_arrival = planet_position(
            arrival_time, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD
        )

        # Simple interpolation with a slight curve
        x = mars_pos_departure[0] + progress * (
            earth_pos_arrival[0] - mars_pos_departure[0]
        )
        y = mars_pos_departure[1] + progress * (
            earth_pos_arrival[1] - mars_pos_departure[1]
        )
        # Add a slight arc to the trajectory
        z = np.sin(np.pi * progress) * 0.2 * AU

    return x, y, z


def determine_mission_phase(t):
    """Determine the mission phase based on time t"""
    if t < EARTH_TO_MARS_DURATION:
        return "earth_to_mars", t
    elif t < EARTH_TO_MARS_DURATION + STAY_ON_MARS:
        return "mars_stay", t - EARTH_TO_MARS_DURATION
    else:
        return "mars_to_earth", t - (EARTH_TO_MARS_DURATION + STAY_ON_MARS)


# Create figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")

# Create the Sun (small, not to scale)
sun_x, sun_y, sun_z = create_sphere(
    SUN_RADIUS * 0.0001
)  # Scaled down for visualization

# Create Earth and Mars (scaled up for visibility)
earth_sphere_x, earth_sphere_y, earth_sphere_z = create_sphere(EARTH_RADIUS * 500)
mars_sphere_x, mars_sphere_y, mars_sphere_z = create_sphere(MARS_RADIUS * 500)

# Spacecraft trail
trail_x, trail_y, trail_z = [], [], []
max_trail_length = 100  # Maximum number of points in the trail

# Plot orbital paths
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit_x = EARTH_ORBIT_RADIUS * np.cos(theta)
earth_orbit_y = EARTH_ORBIT_RADIUS * np.sin(theta)
earth_orbit_z = np.zeros_like(theta)

mars_orbit_x = MARS_ORBIT_RADIUS * np.cos(theta)
mars_orbit_y = MARS_ORBIT_RADIUS * np.sin(theta)
mars_orbit_z = np.zeros_like(theta)

# Initialize plots
sun = ax.plot_surface(sun_x, sun_y, sun_z, color="yellow", alpha=1)
(earth_orbit,) = ax.plot(earth_orbit_x, earth_orbit_y, earth_orbit_z, "b-", alpha=0.3)
(mars_orbit,) = ax.plot(mars_orbit_x, mars_orbit_y, mars_orbit_z, "r-", alpha=0.3)
earth = [
    ax.plot_surface(
        earth_sphere_x, earth_sphere_y, earth_sphere_z, color="blue", alpha=0.8
    )
]
mars = [
    ax.plot_surface(mars_sphere_x, mars_sphere_y, mars_sphere_z, color="red", alpha=0.8)
]
(spacecraft,) = ax.plot([], [], [], "ko", markersize=5)
(spacecraft_trail,) = ax.plot([], [], [], "k-", alpha=0.5)

# Text annotations
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
phase_text = ax.text2D(0.02, 0.90, "", transform=ax.transAxes)

# Set axis limits and labels
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.5 * AU, 0.5 * AU])
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth to Mars and Back Mission Trajectory")


def init():
    """Initialize animation"""
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    spacecraft_trail.set_data([], [])
    spacecraft_trail.set_3d_properties([])
    time_text.set_text("")
    phase_text.set_text("")

    # Remove previous Earth and Mars surfaces
    if earth[0]:
        earth[0].remove()
    if mars[0]:
        mars[0].remove()

    return spacecraft, spacecraft_trail, time_text, phase_text


def update(frame):
    """Update animation for each frame"""
    # Calculate current mission time
    t = frame * TOTAL_MISSION_DURATION / FRAMES

    # Determine mission phase
    phase, phase_time = determine_mission_phase(t)

    # Calculate planet positions
    earth_pos = planet_position(t, EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD)
    mars_pos = planet_position(t, MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD)

    # Calculate spacecraft position
    sc_pos = spacecraft_trajectory(phase_time, phase)

    # Update spacecraft position
    spacecraft.set_data([sc_pos[0]], [sc_pos[1]])
    spacecraft.set_3d_properties([sc_pos[2]])

    # Update trail
    trail_x.append(sc_pos[0])
    trail_y.append(sc_pos[1])
    trail_z.append(sc_pos[2])

    # Limit trail length
    if len(trail_x) > max_trail_length:
        trail_x.pop(0)
        trail_y.pop(0)
        trail_z.pop(0)

    spacecraft_trail.set_data(trail_x, trail_y)
    spacecraft_trail.set_3d_properties(trail_z)

    # Update Earth and Mars positions
    if earth[0]:
        earth[0].remove()
    if mars[0]:
        mars[0].remove()

    earth[0] = ax.plot_surface(
        earth_sphere_x + earth_pos[0],
        earth_sphere_y + earth_pos[1],
        earth_sphere_z + earth_pos[2],
        color="blue",
        alpha=0.8,
    )

    mars[0] = ax.plot_surface(
        mars_sphere_x + mars_pos[0],
        mars_sphere_y + mars_pos[1],
        mars_sphere_z + mars_pos[2],
        color="red",
        alpha=0.8,
    )

    # Update text
    time_text.set_text(f"Mission Day: {int(t)}")

    if phase == "earth_to_mars":
        phase_text.set_text("Phase: Earth to Mars Transit")
    elif phase == "mars_stay":
        phase_text.set_text("Phase: Mars Surface Operations")
    else:
        phase_text.set_text("Phase: Mars to Earth Return")

    return spacecraft, spacecraft_trail, time_text, phase_text, earth[0], mars[0]


# Create animation
ani = FuncAnimation(fig, update, frames=FRAMES, init_func=init, interval=50, blit=False)

# Adjust view angle
ax.view_init(elev=30, azim=45)

plt.tight_layout()
plt.show()

# Uncomment to save animation
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)
