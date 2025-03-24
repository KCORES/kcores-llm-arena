import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from datetime import datetime, timedelta
import matplotlib.colors as colors

# Constants
AU = 149.6e6  # Astronomical Unit in km
EARTH_ORBIT_RADIUS = 1.0 * AU
MARS_ORBIT_RADIUS = 1.524 * AU
EARTH_YEAR_DAYS = 365.25
MARS_YEAR_DAYS = 687
SYNODIC_PERIOD = 779.9  # Earth-Mars synodic period in days

# Mission parameters
LAUNCH_DATE = datetime(2022, 12, 1)
OUTBOUND_DURATION = 210  # days
MARS_STAY_DURATION = SYNODIC_PERIOD - OUTBOUND_DURATION - 200  # days
RETURN_DURATION = 210  # days

# Create figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("black")
fig.patch.set_facecolor("black")

# Set axis limits
limit = 1.8 * AU
ax.set_xlim([-limit, limit])
ax.set_ylim([-limit, limit])
ax.set_zlim([-limit, limit])
ax.set_axis_off()

# Add title with white color
plt.title("Earth-Mars-Earth Mission Trajectory", color="white", fontsize=14)

# Create stars background
np.random.seed(42)
num_stars = 500
stars = np.random.uniform(-limit, limit, (3, num_stars))
ax.scatter(stars[0], stars[1], stars[2], color="white", alpha=0.5, s=0.5)

# Create the Sun
sun_radius = 20
ax.scatter([0], [0], [0], color="yellow", s=sun_radius * 20, alpha=1.0)


# Function to calculate planet positions at a given date
def planet_position(days_since_launch, planet="earth"):
    date = LAUNCH_DATE + timedelta(days=days_since_launch)
    if planet.lower() == "earth":
        orbit_radius = EARTH_ORBIT_RADIUS
        period = EARTH_YEAR_DAYS
    else:  # Mars
        orbit_radius = MARS_ORBIT_RADIUS
        period = MARS_YEAR_DAYS

    # Simplified circular orbit calculation
    angle = 2 * np.pi * (days_since_launch % period) / period
    x = orbit_radius * np.cos(angle)
    y = orbit_radius * np.sin(angle)
    z = 0  # Assuming orbits are in the same plane for simplicity

    return np.array([x, y, z])


# Function to calculate spacecraft position
def spacecraft_position(days_since_launch):
    total_mission_duration = OUTBOUND_DURATION + MARS_STAY_DURATION + RETURN_DURATION

    if days_since_launch < 0:
        return planet_position(0)  # Before launch

    elif days_since_launch <= OUTBOUND_DURATION:
        # Outbound leg (Earth to Mars)
        earth_pos = planet_position(0)
        mars_pos = planet_position(OUTBOUND_DURATION, "mars")
        progress = days_since_launch / OUTBOUND_DURATION
        return earth_pos + (mars_pos - earth_pos) * progress

    elif days_since_launch <= OUTBOUND_DURATION + MARS_STAY_DURATION:
        # Stay at Mars
        return planet_position(days_since_launch, "mars")

    elif days_since_launch <= total_mission_duration:
        # Return leg (Mars to Earth)
        mars_pos = planet_position(OUTBOUND_DURATION + MARS_STAY_DURATION, "mars")
        earth_pos = planet_position(total_mission_duration)
        progress = (
            days_since_launch - (OUTBOUND_DURATION + MARS_STAY_DURATION)
        ) / RETURN_DURATION
        return mars_pos + (earth_pos - mars_pos) * progress

    else:
        # After mission completion
        return planet_position(days_since_launch)


# Initialize trajectory lines
(earth_orbit_line,) = ax.plot([], [], [], "b-", alpha=0.3, linewidth=1)
(mars_orbit_line,) = ax.plot([], [], [], "r-", alpha=0.3, linewidth=1)
(trajectory_line,) = ax.plot([], [], [], "w-", alpha=0.5, linewidth=1)
(current_position,) = ax.plot([], [], [], "yo", markersize=6)

# Initialize planet markers
earth_marker = ax.scatter([], [], [], color="blue", s=100, label="Earth")
mars_marker = ax.scatter([], [], [], color="red", s=80, label="Mars")

# Initialize date text
date_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, color="white")

# Initialize legend
ax.legend(loc="upper right", facecolor="black", labelcolor="white")


# Animation update function
def update(frame):
    days_since_launch = frame * 5  # Speed up animation (5 days per frame)

    # Calculate positions
    earth_pos = planet_position(days_since_launch)
    mars_pos = planet_position(days_since_launch, "mars")
    spacecraft_pos = spacecraft_position(days_since_launch)

    # Update planet markers
    earth_marker._offsets3d = ([earth_pos[0]], [earth_pos[1]], [earth_pos[2]])
    mars_marker._offsets3d = ([mars_pos[0]], [mars_pos[1]], [mars_pos[2]])

    # Update trajectory line
    if days_since_launch <= OUTBOUND_DURATION + MARS_STAY_DURATION + RETURN_DURATION:
        # Calculate points along the trajectory up to current time
        times = np.linspace(
            0,
            min(
                days_since_launch,
                OUTBOUND_DURATION + MARS_STAY_DURATION + RETURN_DURATION,
            ),
            100,
        )
        traj_points = np.array([spacecraft_position(t) for t in times])
        trajectory_line.set_data(traj_points[:, 0], traj_points[:, 1])
        trajectory_line.set_3d_properties(traj_points[:, 2])

    # Update current position marker
    current_position.set_data([spacecraft_pos[0]], [spacecraft_pos[1]])
    current_position.set_3d_properties([spacecraft_pos[2]])

    # Update orbits (static for this simplified model)
    theta = np.linspace(0, 2 * np.pi, 100)
    earth_orbit_x = EARTH_ORBIT_RADIUS * np.cos(theta)
    earth_orbit_y = EARTH_ORBIT_RADIUS * np.sin(theta)
    earth_orbit_line.set_data(earth_orbit_x, earth_orbit_y)
    earth_orbit_line.set_3d_properties(np.zeros_like(theta))

    mars_orbit_x = MARS_ORBIT_RADIUS * np.cos(theta)
    mars_orbit_y = MARS_ORBIT_RADIUS * np.sin(theta)
    mars_orbit_line.set_data(mars_orbit_x, mars_orbit_y)
    mars_orbit_line.set_3d_properties(np.zeros_like(theta))

    # Update date text
    current_date = LAUNCH_DATE + timedelta(days=days_since_launch)
    mission_phase = "Pre-launch"
    if 0 <= days_since_launch <= OUTBOUND_DURATION:
        mission_phase = "Outbound to Mars"
    elif (
        OUTBOUND_DURATION < days_since_launch <= OUTBOUND_DURATION + MARS_STAY_DURATION
    ):
        mission_phase = "At Mars"
    elif (
        OUTBOUND_DURATION + MARS_STAY_DURATION
        < days_since_launch
        <= OUTBOUND_DURATION + MARS_STAY_DURATION + RETURN_DURATION
    ):
        mission_phase = "Return to Earth"
    else:
        mission_phase = "Mission Complete"

    date_text.set_text(
        f"Date: {current_date.strftime('%Y-%m-%d')}\nMission Phase: {mission_phase}"
    )

    # Adjust view angle for better visualization
    ax.view_init(elev=20, azim=frame / 5)

    return (
        earth_marker,
        mars_marker,
        trajectory_line,
        current_position,
        earth_orbit_line,
        mars_orbit_line,
        date_text,
    )


# Calculate total frames needed
total_days = OUTBOUND_DURATION + MARS_STAY_DURATION + RETURN_DURATION
total_frames = int(total_days / 5) + 10  # Add some extra frames

# Create animation
ani = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)

plt.tight_layout()

# To save the animation (uncomment if needed)
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)

plt.show()
