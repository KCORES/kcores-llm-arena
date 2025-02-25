import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

# Constants (in astronomical units and normalized time)
SUN_SIZE = 0.2
EARTH_SIZE = 0.05
MARS_SIZE = 0.04
SPACECRAFT_SIZE = 0.01

# Orbital parameters
EARTH_RADIUS = 1.0  # 1 AU
MARS_RADIUS = 1.52  # 1.52 AU
EARTH_PERIOD = 365  # days
MARS_PERIOD = 687  # days

# Time parameters for animation
LAUNCH_TIME = 0
MARS_ARRIVAL_TIME = 260  # days
MARS_DEPARTURE_TIME = MARS_ARRIVAL_TIME + 500  # Stay on Mars
EARTH_RETURN_TIME = MARS_DEPARTURE_TIME + 260
TOTAL_TIME = EARTH_RETURN_TIME + 50


def create_orbit(radius, num_points=100):
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = np.zeros_like(theta)
    return x, y, z


def planet_position(t, radius, period):
    angle = 2 * np.pi * (t % period) / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0
    return x, y, z


def spacecraft_trajectory(t, phase="earth_to_mars"):
    if phase == "earth_to_mars":
        # Start and end positions
        start_t = LAUNCH_TIME
        end_t = MARS_ARRIVAL_TIME

        if t <= start_t:
            # At Earth before launch
            return planet_position(t, EARTH_RADIUS, EARTH_PERIOD)

        elif t >= end_t:
            # At Mars after arrival
            return planet_position(t, MARS_RADIUS, MARS_PERIOD)

        else:
            # In transit - create a curved trajectory
            progress = (t - start_t) / (end_t - start_t)

            # Get Earth and Mars positions at start and end times
            earth_x, earth_y, earth_z = planet_position(
                start_t, EARTH_RADIUS, EARTH_PERIOD
            )
            mars_x, mars_y, mars_z = planet_position(end_t, MARS_RADIUS, MARS_PERIOD)

            # Create curved path using quadratic Bezier curve
            # Control point above the orbital plane
            control_x = (earth_x + mars_x) / 2
            control_y = (earth_y + mars_y) / 2
            control_z = 0.3  # Height above orbital plane

            # Quadratic Bezier formula
            x = (
                (1 - progress) ** 2 * earth_x
                + 2 * (1 - progress) * progress * control_x
                + progress**2 * mars_x
            )
            y = (
                (1 - progress) ** 2 * earth_y
                + 2 * (1 - progress) * progress * control_y
                + progress**2 * mars_y
            )
            z = (
                (1 - progress) ** 2 * earth_z
                + 2 * (1 - progress) * progress * control_z
                + progress**2 * mars_z
            )

            return x, y, z

    elif phase == "mars_to_earth":
        # Start and end positions
        start_t = MARS_DEPARTURE_TIME
        end_t = EARTH_RETURN_TIME

        if t <= start_t:
            # At Mars before departure
            return planet_position(t, MARS_RADIUS, MARS_PERIOD)

        elif t >= end_t:
            # At Earth after arrival
            return planet_position(t, EARTH_RADIUS, EARTH_PERIOD)

        else:
            # In transit - create a curved trajectory
            progress = (t - start_t) / (end_t - start_t)

            # Get Mars and Earth positions at start and end times
            mars_x, mars_y, mars_z = planet_position(start_t, MARS_RADIUS, MARS_PERIOD)
            earth_x, earth_y, earth_z = planet_position(
                end_t, EARTH_RADIUS, EARTH_PERIOD
            )

            # Create curved path using quadratic Bezier curve
            # Control point above the orbital plane
            control_x = (mars_x + earth_x) / 2
            control_y = (mars_y + earth_y) / 2
            control_z = (
                -0.3
            )  # Height below orbital plane (different path than outbound)

            # Quadratic Bezier formula
            x = (
                (1 - progress) ** 2 * mars_x
                + 2 * (1 - progress) * progress * control_x
                + progress**2 * earth_x
            )
            y = (
                (1 - progress) ** 2 * mars_y
                + 2 * (1 - progress) * progress * control_y
                + progress**2 * earth_y
            )
            z = (
                (1 - progress) ** 2 * mars_z
                + 2 * (1 - progress) * progress * control_z
                + progress**2 * earth_z
            )

            return x, y, z


def get_spacecraft_position(t):
    if t < MARS_ARRIVAL_TIME:
        return spacecraft_trajectory(t, "earth_to_mars")
    elif t < MARS_DEPARTURE_TIME:
        return planet_position(t, MARS_RADIUS, MARS_PERIOD)
    else:
        return spacecraft_trajectory(t, "mars_to_earth")


def get_spacecraft_trail(current_time, num_points=50):
    """Get trail of positions leading up to current time for spacecraft"""
    times = np.linspace(max(0, current_time - 100), current_time, num_points)
    positions = [get_spacecraft_position(t) for t in times]
    x_trail = [pos[0] for pos in positions]
    y_trail = [pos[1] for pos in positions]
    z_trail = [pos[2] for pos in positions]
    return x_trail, y_trail, z_trail


# Create the figure and 3D axis
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection="3d")

# Create orbits
earth_orbit_x, earth_orbit_y, earth_orbit_z = create_orbit(EARTH_RADIUS)
mars_orbit_x, mars_orbit_y, mars_orbit_z = create_orbit(MARS_RADIUS)

# Initial plot components
sun = ax.scatter(
    [0],
    [0],
    [0],
    s=SUN_SIZE * 1000,
    c="yellow",
    edgecolor="orange",
    alpha=1,
    label="Sun",
)
(earth_orbit,) = ax.plot(earth_orbit_x, earth_orbit_y, earth_orbit_z, "b-", alpha=0.3)
(mars_orbit,) = ax.plot(mars_orbit_x, mars_orbit_y, mars_orbit_z, "r-", alpha=0.3)

# Initial positions
earth_pos = ax.scatter(
    [], [], [], s=EARTH_SIZE * 1000, c="blue", edgecolor="blue", label="Earth"
)
mars_pos = ax.scatter(
    [], [], [], s=MARS_SIZE * 1000, c="red", edgecolor="red", label="Mars"
)
spacecraft = ax.scatter(
    [],
    [],
    [],
    s=SPACECRAFT_SIZE * 1000,
    c="white",
    edgecolor="black",
    label="Spacecraft",
)
(spacecraft_trail,) = ax.plot([], [], [], "k-", alpha=0.4)

# Mission phases text display
mission_text = ax.text2D(
    0.02, 0.95, "", transform=ax.transAxes, fontsize=12, color="white"
)

# Text to show days elapsed
time_text = ax.text2D(
    0.02, 0.90, "", transform=ax.transAxes, fontsize=12, color="white"
)

# Set visualization parameters
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.5, 0.5)
ax.legend(loc="upper right")

# Set a black background
ax.set_facecolor("black")
fig.patch.set_facecolor("black")
ax.grid(False)
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False

# Title
plt.title("Earth to Mars Round Trip Mission", color="white", fontsize=16)


def animate(i):
    # Current time in days
    t = i * (TOTAL_TIME / 500)  # 500 frames for the total mission

    # Update planet positions
    earth_x, earth_y, earth_z = planet_position(t, EARTH_RADIUS, EARTH_PERIOD)
    mars_x, mars_y, mars_z = planet_position(t, MARS_RADIUS, MARS_PERIOD)

    earth_pos._offsets3d = ([earth_x], [earth_y], [earth_z])
    mars_pos._offsets3d = ([mars_x], [mars_y], [mars_z])

    # Update spacecraft position
    sc_x, sc_y, sc_z = get_spacecraft_position(t)
    spacecraft._offsets3d = ([sc_x], [sc_y], [sc_z])

    # Update trail
    x_trail, y_trail, z_trail = get_spacecraft_trail(t)
    spacecraft_trail.set_data(x_trail, y_trail)
    spacecraft_trail.set_3d_properties(z_trail)

    # Update mission phase text
    if t < LAUNCH_TIME:
        phase = "Preparing for Launch"
    elif t < MARS_ARRIVAL_TIME:
        phase = "Earth to Mars Transit"
    elif t < MARS_DEPARTURE_TIME:
        phase = "On Mars Surface"
    elif t < EARTH_RETURN_TIME:
        phase = "Mars to Earth Transit"
    else:
        phase = "Mission Complete"

    mission_text.set_text(f"Mission Phase: {phase}")
    time_text.set_text(f"Days Elapsed: {int(t)}")

    return earth_pos, mars_pos, spacecraft, spacecraft_trail, mission_text, time_text


# Create animation
ani = FuncAnimation(fig, animate, frames=500, interval=50, blit=True)

# Uncomment to save the animation (requires ffmpeg or imagemagick)
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=150)

plt.tight_layout()
plt.show()
