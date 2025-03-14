import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from astropy.time import Time
import astropy.units as u
from astropy.coordinates import get_body, EarthLocation, SkyCoord


def calculate_trajectory(
    start_time, duration, earth_radius=6371 * u.km, mars_radius=3389.5 * u.km
):
    """
    Generates a simplified trajectory for a launch to Mars and back.
    This is a highly simplified trajectory and doesn't account for orbital mechanics details.

    Args:
        start_time (astropy.time.Time):  The start time of the launch.
        duration (float):  The total duration of the mission (in days).

    Returns:
        tuple: A tuple containing three numpy arrays representing the x, y, and z coordinates of the trajectory.
    """

    num_points = 500
    t = np.linspace(0, duration, num_points)

    # Simplified trajectory - parabolic arcs
    earth_to_mars_time = duration / 2
    mars_to_earth_time = duration / 2

    # Earth to Mars phase
    x1 = np.linspace(
        0, 225e6, int(num_points * earth_to_mars_time / duration)
    )  # Distance Earth-Mars in km
    y1 = (x1 / (225e6 / 2)) ** 2 - 225e6
    z1 = np.zeros_like(x1)

    # Mars to Earth phase (mirrored trajectory)
    x2 = np.linspace(225e6, 0, int(num_points * mars_to_earth_time / duration))
    y2 = (x2 / (225e6 / 2)) ** 2 - 225e6
    z2 = np.zeros_like(x2)

    x = np.concatenate((x1, x2))
    y = np.concatenate((y1, y2))
    z = np.concatenate((z1, z2))

    return x, y, z


def animate_trajectory(i):
    """
    Animation function to update the plot for each frame.
    """

    global x, y, z, line

    # Calculate the current position based on the frame number
    index = int(i * len(x) / total_frames)
    x_current = x[:index]
    y_current = y[:index]
    z_current = z[:index]

    line.set_data(x_current, y_current, z_current)

    # Update Earth and Mars positions
    earth_pos = get_body("earth", start_time + (i * dt))
    mars_pos = get_body("mars", start_time + (i * dt))

    earth.set_data(
        [earth_pos.x.to(u.km).value],
        [earth_pos.y.to(u.km).value],
        [earth_pos.z.to(u.km).value],
    )
    mars.set_data(
        [mars_pos.x.to(u.km).value],
        [mars_pos.y.to(u.km).value],
        [mars_pos.z.to(u.km).value],
    )

    return line, earth, mars


# Define parameters
duration = 500  # Days
start_time = Time("2024-01-01T00:00:00")  # Example start time
dt = timedelta(days=1)
total_frames = duration * 24 * 60  # Number of frames for a smooth animation

# Calculate Trajectory
x, y, z = calculate_trajectory(start_time, duration)

# Create Figure and Axes
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")

# --- Setup Earth/Mars ---
earth_pos = get_body("earth", start_time)
mars_pos = get_body("mars", start_time)


(earth,) = ax.plot(
    [earth_pos.x.to(u.km).value],
    [earth_pos.y.to(u.km).value],
    [earth_pos.z.to(u.km).value],
    "bo",
    markersize=10,
    label="Earth",
)
(mars,) = ax.plot(
    [mars_pos.x.to(u.km).value],
    [mars_pos.y.to(u.km).value],
    [mars_pos.z.to(u.km).value],
    "ro",
    markersize=7,
    label="Mars",
)

# Initialize trajectory line
(line,) = ax.plot([], [], [], "g-", lw=2, label="Trajectory")

# Set axis limits (adjust as needed)
max_distance = max(np.max(np.abs(x)), np.max(np.abs(y)), np.max(np.abs(z)))
ax.set_xlim([-max_distance * 1.1, max_distance * 1.1])
ax.set_ylim([-max_distance * 1.1, max_distance * 1.1])
ax.set_zlim([-max_distance * 1.1, max_distance * 1.1])

ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Simplified Earth to Mars and Back Trajectory")
ax.legend()
ax.view_init(elev=20, azim=45)  # Adjust viewing angle

# Create Animation
ani = animation.FuncAnimation(
    fig, animate_trajectory, frames=total_frames, interval=20, blit=True
)

# Save animation (optional) - requires ffmpeg or similar encoding
# ani.save("earth_mars_trajectory.mp4", writer="ffmpeg", fps=30)

plt.show()
