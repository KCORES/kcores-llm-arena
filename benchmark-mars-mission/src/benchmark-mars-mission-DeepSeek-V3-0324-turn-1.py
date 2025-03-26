import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from datetime import datetime, timedelta
from astropy.time import Time
from astropy import units as u
from astropy.coordinates import solar_system_ephemeris, get_body_barycentric_posvel

# Set up the figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")

# Constants
AU = 1.495978707e11  # Astronomical Unit in meters
scale = 1.5  # Scale factor for visualization

# Set up the solar system bodies
solar_system_ephemeris.set("de432s")


# Function to get planet positions at a given date
def get_planet_positions(date):
    time = Time(date)
    earth_pos, _ = get_body_barycentric_posvel("earth", time)
    mars_pos, _ = get_body_barycentric_posvel("mars", time)
    return earth_pos.xyz.to(u.AU).value, mars_pos.xyz.to(u.AU).value


# Mission parameters
departure_date = datetime(2022, 12, 1)
mars_stay_duration = timedelta(days=450)
return_departure_date = (
    departure_date + mars_stay_duration + timedelta(days=210)
)  # Time to reach Mars + stay

# Time points for the animation
total_days = 900  # Total mission duration in days
time_points = [departure_date + timedelta(days=i) for i in range(total_days)]

# Calculate positions
earth_positions = []
mars_positions = []
spacecraft_positions = []

for i, date in enumerate(time_points):
    earth_pos, mars_pos = get_planet_positions(date)
    earth_positions.append(earth_pos)
    mars_positions.append(mars_pos)

    # Simplified trajectory model (in reality would use Lambert's problem solution)
    if date < departure_date + timedelta(days=210):  # Outbound trip
        progress = min(1, i / 210)
        spacecraft_pos = earth_pos * (1 - progress) + mars_pos * progress
    elif date < return_departure_date:  # Mars stay
        spacecraft_pos = mars_pos
    else:  # Return trip
        return_progress = min(
            1, (i - (return_departure_date - departure_date).days) / 210
        )
        earth_pos_return, _ = get_planet_positions(date)
        spacecraft_pos = (
            mars_pos * (1 - return_progress) + earth_pos_return * return_progress
        )

    spacecraft_positions.append(spacecraft_pos)

earth_positions = np.array(earth_positions)
mars_positions = np.array(mars_positions)
spacecraft_positions = np.array(spacecraft_positions)

# Initialize plot elements
(earth_point,) = ax.plot([], [], [], "o", color="blue", markersize=10, label="Earth")
(mars_point,) = ax.plot([], [], [], "o", color="red", markersize=8, label="Mars")
(sun_point,) = ax.plot([0], [0], [0], "o", color="yellow", markersize=15, label="Sun")
(trajectory_line,) = ax.plot([], [], [], "b-", linewidth=1, alpha=0.5)
(spacecraft_point,) = ax.plot(
    [], [], [], "o", color="black", markersize=5, label="Spacecraft"
)

# Add current position markers
(current_earth,) = ax.plot([], [], [], "o", color="blue", markersize=15, alpha=0.3)
(current_mars,) = ax.plot([], [], [], "o", color="red", markersize=12, alpha=0.3)

# Add orbit circles for Earth and Mars (simplified as circles)
earth_orbit = Circle((0, 0), 1, fill=False, color="blue", alpha=0.3)
mars_orbit = Circle((0, 0), 1.52, fill=False, color="red", alpha=0.3)
ax.add_patch(earth_orbit)
ax.add_patch(mars_orbit)
art3d.pathpatch_2d_to_3d(earth_orbit, z=0, zdir="z")
art3d.pathpatch_2d_to_3d(mars_orbit, z=0, zdir="z")


# Set up the animation
def init():
    ax.set_xlim(-scale * 2, scale * 2)
    ax.set_ylim(-scale * 2, scale * 2)
    ax.set_zlim(-scale * 1, scale * 1)
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Earth-Mars-Earth Mission Trajectory")
    ax.legend(loc="upper right")
    return (
        earth_point,
        mars_point,
        sun_point,
        trajectory_line,
        spacecraft_point,
        current_earth,
        current_mars,
    )


def update(frame):
    # Update planet positions
    earth_point.set_data([earth_positions[frame, 0]], [earth_positions[frame, 1]])
    earth_point.set_3d_properties([earth_positions[frame, 2]])

    mars_point.set_data([mars_positions[frame, 0]], [mars_positions[frame, 1]])
    mars_point.set_3d_properties([mars_positions[frame, 2]])

    # Update spacecraft position and trajectory
    trajectory_line.set_data(
        spacecraft_positions[: frame + 1, 0], spacecraft_positions[: frame + 1, 1]
    )
    trajectory_line.set_3d_properties(spacecraft_positions[: frame + 1, 2])

    spacecraft_point.set_data(
        [spacecraft_positions[frame, 0]], [spacecraft_positions[frame, 1]]
    )
    spacecraft_point.set_3d_properties([spacecraft_positions[frame, 2]])

    # Update current position markers
    current_earth.set_data([earth_positions[frame, 0]], [earth_positions[frame, 1]])
    current_earth.set_3d_properties([earth_positions[frame, 2]])

    current_mars.set_data([mars_positions[frame, 0]], [mars_positions[frame, 1]])
    current_mars.set_3d_properties([mars_positions[frame, 2]])

    # Update title with date
    ax.set_title(
        f'Earth-Mars-Earth Mission\nDate: {time_points[frame].strftime("%Y-%m-%d")}'
    )

    return (
        earth_point,
        mars_point,
        trajectory_line,
        spacecraft_point,
        current_earth,
        current_mars,
    )


# Create animation
ani = FuncAnimation(
    fig,
    update,
    frames=len(time_points),
    init_func=init,
    blit=False,
    interval=50,
    repeat=True,
)

plt.tight_layout()
plt.show()

# To save the animation (uncomment to use)
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)
