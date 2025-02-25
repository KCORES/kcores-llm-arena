import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants (in astronomical units and days)
EARTH_ORBIT_RADIUS = 1.0  # AU
MARS_ORBIT_RADIUS = 1.524  # AU
EARTH_ORBIT_PERIOD = 365.25  # days
MARS_ORBIT_PERIOD = 687  # days

# Create figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")


# Calculate orbital positions
def orbit_position(radius, period, days, phase=0):
    angle = 2 * np.pi * days / period + phase
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0  # Simplification: planets orbit in the same plane
    return x, y, z


# Initial positions
days = np.linspace(0, 1000, 1000)  # 1000 days of simulation
earth_x, earth_y, earth_z = [], [], []
mars_x, mars_y, mars_z = [], [], []

# Mars phase offset (angle between Earth and Mars at simulation start)
mars_phase = 1.5  # Arbitrary starting offset

# Calculate orbits
for day in days:
    ex, ey, ez = orbit_position(EARTH_ORBIT_RADIUS, EARTH_ORBIT_PERIOD, day)
    mx, my, mz = orbit_position(MARS_ORBIT_RADIUS, MARS_ORBIT_PERIOD, day, mars_phase)

    earth_x.append(ex)
    earth_y.append(ey)
    earth_z.append(ez)

    mars_x.append(mx)
    mars_y.append(my)
    mars_z.append(mz)

# Draw orbits
(earth_orbit,) = ax.plot(earth_x, earth_y, earth_z, "b-", alpha=0.3)
(mars_orbit,) = ax.plot(mars_x, mars_y, mars_z, "r-", alpha=0.3)

# Plot the Sun
ax.scatter([0], [0], [0], color="yellow", s=200, label="Sun")

# Initial planet positions
(earth_point,) = ax.plot(
    [earth_x[0]], [earth_y[0]], [earth_z[0]], "bo", markersize=8, label="Earth"
)
(mars_point,) = ax.plot(
    [mars_x[0]], [mars_y[0]], [mars_z[0]], "ro", markersize=6, label="Mars"
)

# Spacecraft trajectory
# Earth to Mars (first 250 days) - Hohmann transfer approximation
outbound_days = np.linspace(0, 250, 250)
traj_x1, traj_y1, traj_z1 = [], [], []

# Return journey (next 250 days)
return_days = np.linspace(250, 500, 250)
traj_x2, traj_y2, traj_z2 = [], [], []

# Calculate simplified spacecraft trajectories
for i, day in enumerate(outbound_days):
    # Interpolation factor (0 to 1) for the journey progress
    t = i / (len(outbound_days) - 1)

    # Get positions of Earth at departure and Mars at arrival
    earth_pos = (earth_x[0], earth_y[0], earth_z[0])
    mars_pos = (mars_x[250], mars_y[250], mars_z[250])

    # Simple curved path approximation (not physically accurate but visually pleasing)
    x = earth_pos[0] + t * (mars_pos[0] - earth_pos[0])
    y = earth_pos[1] + t * (mars_pos[1] - earth_pos[1])

    # Add a curved path by adding a sin function to the z-coordinate
    curve_factor = 0.2 * np.sin(np.pi * t)
    z = earth_pos[2] + t * (mars_pos[2] - earth_pos[2]) + curve_factor

    traj_x1.append(x)
    traj_y1.append(y)
    traj_z1.append(z)

# Calculate return trajectory (Mars to Earth)
for i, day in enumerate(return_days):
    # Interpolation factor (0 to 1) for the journey progress
    t = i / (len(return_days) - 1)

    # Get positions of Mars at departure and Earth at arrival
    mars_pos = (mars_x[250], mars_y[250], mars_z[250])
    earth_pos = (earth_x[500], earth_y[500], earth_z[500])

    # Simple curved path approximation
    x = mars_pos[0] + t * (earth_pos[0] - mars_pos[0])
    y = mars_pos[1] + t * (earth_pos[1] - mars_pos[1])

    # Add a curved path by adding a sin function to the z-coordinate
    curve_factor = 0.2 * np.sin(np.pi * t)
    z = mars_pos[2] + t * (earth_pos[2] - mars_pos[2]) + curve_factor

    traj_x2.append(x)
    traj_y2.append(y)
    traj_z2.append(z)

# Initialize trajectory lines
(outbound_traj,) = ax.plot([], [], [], "g-", label="Earth to Mars")
(return_traj,) = ax.plot([], [], [], "y-", label="Mars to Earth")

# Spacecraft representation
(spacecraft,) = ax.plot([], [], [], "k^", markersize=8, label="Spacecraft")

# Set axis properties
ax.set_xlim([-1.8, 1.8])
ax.set_ylim([-1.8, 1.8])
ax.set_zlim([-0.5, 0.5])
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars-Earth Mission Trajectory")
ax.legend()


# Animation update function
def update(frame):
    # Update Earth and Mars positions
    earth_point.set_data([earth_x[frame]], [earth_y[frame]])
    earth_point.set_3d_properties([earth_z[frame]])

    mars_point.set_data([mars_x[frame]], [mars_y[frame]])
    mars_point.set_3d_properties([mars_z[frame]])

    # Update spacecraft position and trajectory
    if frame < 250:
        # Outbound journey to Mars
        outbound_traj.set_data(traj_x1[: frame + 1], traj_y1[: frame + 1])
        outbound_traj.set_3d_properties(traj_z1[: frame + 1])

        # Show spacecraft
        spacecraft.set_data([traj_x1[frame]], [traj_y1[frame]])
        spacecraft.set_3d_properties([traj_z1[frame]])

        # Don't show return trajectory yet
        return_traj.set_data([], [])
        return_traj.set_3d_properties([])

    elif frame < 500:
        # Show full outbound trajectory
        outbound_traj.set_data(traj_x1, traj_y1)
        outbound_traj.set_3d_properties(traj_z1)

        # Return journey from Mars
        return_frame = frame - 250
        return_traj.set_data(traj_x2[: return_frame + 1], traj_y2[: return_frame + 1])
        return_traj.set_3d_properties(traj_z2[: return_frame + 1])

        # Show spacecraft
        spacecraft.set_data([traj_x2[return_frame]], [traj_y2[return_frame]])
        spacecraft.set_3d_properties([traj_z2[return_frame]])

    else:
        # Show complete trajectories
        outbound_traj.set_data(traj_x1, traj_y1)
        outbound_traj.set_3d_properties(traj_z1)

        return_traj.set_data(traj_x2, traj_y2)
        return_traj.set_3d_properties(traj_z2)

        # Position the spacecraft at the end of its journey
        spacecraft.set_data(
            [earth_x[frame % len(earth_x)]], [earth_y[frame % len(earth_y)]]
        )
        spacecraft.set_3d_properties([earth_z[frame % len(earth_z)]])

    # Rotate the view for a dynamic perspective
    ax.view_init(elev=20, azim=frame / 5)

    return earth_point, mars_point, outbound_traj, return_traj, spacecraft


# Create animation
ani = FuncAnimation(fig, update, frames=600, interval=50, blit=True)

plt.tight_layout()
plt.show()

# Uncomment to save the animation
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)
