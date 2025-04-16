import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants and parameters
AU = 1.496e8  # km (average distance Earth-Sun)
earth_orbit_radius = 1 * AU  # in km
mars_orbit_radius = 1.524 * AU

earth_orbit_period = 365  # days (simplified)
mars_orbit_period = 687  # days (simplified)

total_days = 700  # total simulation time in days
dt = 1  # day timestep


# Convert days to radians for position calculation
def orbit_pos(radius, period, day):
    angle = 2 * np.pi * (day % period) / period
    return np.array([radius * np.cos(angle), radius * np.sin(angle), 0])


# Time windows for events (approximate)
launch_day = 0
arrival_day = 260  # Earth to Mars transfer (~8.5 months)
stay_duration = 100  # time on Mars before return launch
return_launch_day = arrival_day + stay_duration
return_arrival_day = return_launch_day + 260

# Setup figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2 * AU, 2 * AU)
ax.set_ylim(-2 * AU, 2 * AU)
ax.set_zlim(-0.5 * AU, 0.5 * AU)
ax.set_facecolor("black")
plt.style.use("dark_background")

# Plot the Sun
ax.scatter(0, 0, 0, color="yellow", s=250, label="Sun")

# Draw orbits
theta = np.linspace(0, 2 * np.pi, 300)
earth_orbit_x = earth_orbit_radius * np.cos(theta)
earth_orbit_y = earth_orbit_radius * np.sin(theta)
mars_orbit_x = mars_orbit_radius * np.cos(theta)
mars_orbit_y = mars_orbit_radius * np.sin(theta)

(earth_orbit_line,) = ax.plot(
    earth_orbit_x, earth_orbit_y, 0 * theta, ls="--", color="blue", label="Earth Orbit"
)
(mars_orbit_line,) = ax.plot(
    mars_orbit_x, mars_orbit_y, 0 * theta, ls="--", color="red", label="Mars Orbit"
)

# Plot Earth and Mars initial positions
(earth_dot,) = ax.plot([], [], [], "o", color="blue", markersize=12, label="Earth")
(mars_dot,) = ax.plot([], [], [], "o", color="red", markersize=10, label="Mars")

# Spacecraft plot
(spacecraft_dot,) = ax.plot(
    [], [], [], "o", color="white", markersize=5, label="Spacecraft"
)

# Labels and legend
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.view_init(elev=30, azim=30)
ax.legend(loc="upper right")


def interpolate(p1, p2, fraction):
    """Linear interpolation between points p1 and p2."""
    return p1 + fraction * (p2 - p1)


def update(day):
    # Update Earth and Mars position
    earth_pos = orbit_pos(earth_orbit_radius, earth_orbit_period, day)
    mars_pos = orbit_pos(mars_orbit_radius, mars_orbit_period, day)

    earth_dot.set_data(earth_pos[0], earth_pos[1])
    earth_dot.set_3d_properties(earth_pos[2])

    mars_dot.set_data(mars_pos[0], mars_pos[1])
    mars_dot.set_3d_properties(mars_pos[2])

    # Spacecraft trajectory phases:
    # 1. On Earth (day < launch_day)
    # 2. Earth to Mars transfer (launch_day <= day < arrival_day)
    # 3. Landed on Mars (arrival_day <= day < return_launch_day)
    # 4. Mars to Earth transfer (return_launch_day <= day < return_arrival_day)
    # 5. Landed on Earth (day >= return_arrival_day)

    if day < launch_day:
        # Before launch, spacecraft on Earth
        spacecraft_pos = earth_pos
    elif launch_day <= day < arrival_day:
        # Earth to Mars travel
        fraction = (day - launch_day) / (arrival_day - launch_day)
        # Linear interpolation between Earth pos at launch and Mars pos at arrival
        earth_launch_pos = orbit_pos(earth_orbit_radius, earth_orbit_period, launch_day)
        mars_arrival_pos = orbit_pos(mars_orbit_radius, mars_orbit_period, arrival_day)
        spacecraft_pos = interpolate(earth_launch_pos, mars_arrival_pos, fraction)
    elif arrival_day <= day < return_launch_day:
        # Spacecraft on Mars surface
        spacecraft_pos = orbit_pos(mars_orbit_radius, mars_orbit_period, arrival_day)
    elif return_launch_day <= day < return_arrival_day:
        # Mars to Earth return travel
        fraction = (day - return_launch_day) / (return_arrival_day - return_launch_day)
        mars_launch_pos = orbit_pos(
            mars_orbit_radius, mars_orbit_period, return_launch_day
        )
        earth_arrival_pos = orbit_pos(
            earth_orbit_radius, earth_orbit_period, return_arrival_day
        )
        spacecraft_pos = interpolate(mars_launch_pos, earth_arrival_pos, fraction)
    else:
        # Spacecraft on Earth
        spacecraft_pos = orbit_pos(
            earth_orbit_radius, earth_orbit_period, return_arrival_day
        )

    spacecraft_dot.set_data(spacecraft_pos[0], spacecraft_pos[1])
    spacecraft_dot.set_3d_properties(spacecraft_pos[2])

    title_str = f"Day {day}"
    if day < launch_day:
        title_str += " - Pre-launch on Earth"
    elif launch_day <= day < arrival_day:
        title_str += " - Traveling to Mars"
    elif arrival_day <= day < return_launch_day:
        title_str += " - Landed on Mars"
    elif return_launch_day <= day < return_arrival_day:
        title_str += " - Returning to Earth"
    else:
        title_str += " - Landed on Earth"

    ax.set_title(title_str, color="white")

    return earth_dot, mars_dot, spacecraft_dot


# Create animation
anim = FuncAnimation(
    fig, update, frames=range(0, total_days, dt), interval=50, blit=False
)

plt.show()
