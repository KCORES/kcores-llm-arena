import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import for 3D plotting
from matplotlib.animation import FuncAnimation

# Number of frames for each leg of the mission
num_points_outbound = 150  # Frames for Earth to Mars transfer
dwell_frames = 30  # Frames where the spacecraft remains at Mars
num_points_inbound = 150  # Frames for Mars to Earth transfer
total_frames = num_points_outbound + dwell_frames + num_points_inbound


# Function to compute a curved trajectory between two points using a sinusoidal vertical displacement
def compute_trajectory(P0, P1, height, num_points):
    """
    Compute a curved trajectory from P0 to P1.

    Parameters:
        P0 : numpy array
            Starting position (3D).
        P1 : numpy array
            Ending position (3D).
        height : float
            Maximum vertical offset (along z-axis) applied as a sine function.
        num_points : int
            Number of discretized points along the trajectory.

    Returns:
        traj : numpy array
            Array of shape (num_points, 3) containing trajectory coordinates.
    """
    t = np.linspace(0, 1, num_points)
    # Linear interpolation between P0 and P1 plus a vertical (z-axis) sinusoidal offset
    traj = (
        (1 - t)[:, None] * P0
        + t[:, None] * P1
        + np.outer(np.sin(np.pi * t), np.array([0, 0, height]))
    )
    return traj


# Define fixed positions for Earth and Mars (assuming they在发射窗口时刻处于同一直线上)
earth_pos = np.array([1.0, 0, 0])  # Earth position at departure
mars_pos = np.array([1.523, 0, 0])  # Mars position at arrival

# Compute trajectories for outbound (Earth->Mars) and inbound (Mars->Earth) legs
traj_out = compute_trajectory(
    earth_pos, mars_pos, height=0.4, num_points=num_points_outbound
)
traj_in = compute_trajectory(
    mars_pos, earth_pos, height=-0.4, num_points=num_points_inbound
)

# Setup the 3D plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_title("Interplanetary Transfer Trajectory", fontsize=14)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-1, 1])

# Plot the Sun at the origin
ax.scatter(0, 0, 0, color="yellow", s=300, label="Sun")

# Plot Earth's orbit (circle in the x-y plane)
theta = np.linspace(0, 2 * np.pi, 100)
x_orbit_earth = np.cos(theta)
y_orbit_earth = np.sin(theta)
z_orbit_earth = np.zeros_like(theta)
ax.plot(
    x_orbit_earth,
    y_orbit_earth,
    z_orbit_earth,
    color="blue",
    linestyle=":",
    label="Earth Orbit",
)

# Plot Mars' orbit (circle in the x-y plane)
x_orbit_mars = 1.523 * np.cos(theta)
y_orbit_mars = 1.523 * np.sin(theta)
z_orbit_mars = np.zeros_like(theta)
ax.plot(
    x_orbit_mars,
    y_orbit_mars,
    z_orbit_mars,
    color="red",
    linestyle=":",
    label="Mars Orbit",
)

# Plot Earth and Mars as stationary markers at the transfer window positions
(earth_marker,) = ax.plot(
    [earth_pos[0]],
    [earth_pos[1]],
    [earth_pos[2]],
    marker="o",
    color="blue",
    markersize=10,
    label="Earth",
)
(mars_marker,) = ax.plot(
    [mars_pos[0]],
    [mars_pos[1]],
    [mars_pos[2]],
    marker="o",
    color="red",
    markersize=10,
    label="Mars",
)

# Plot reference trajectories as dashed lines for visual guidance
ax.plot(
    traj_out[:, 0],
    traj_out[:, 1],
    traj_out[:, 2],
    color="gray",
    linestyle="--",
    label="Outbound Trajectory",
)
ax.plot(
    traj_in[:, 0],
    traj_in[:, 1],
    traj_in[:, 2],
    color="gray",
    linestyle="--",
    label="Inbound Trajectory",
)

# Initialize a marker for the spacecraft (which会在动画中更新它的位置)
(spacecraft,) = ax.plot(
    [], [], [], marker="o", color="black", markersize=8, label="Spacecraft"
)

# Add a text annotation to show the current phase/status of the mission
status_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)


# Animation update function：依照当前帧数更新航天器的位置及状态文本
def update(frame):
    if frame < num_points_outbound:
        pos = traj_out[frame]
        status = "Outbound: Earth to Mars"
    elif frame < num_points_outbound + dwell_frames:
        pos = traj_out[-1]  # Stay at Mars during dwell phase
        status = "Mars Dwell"
    else:
        pos = traj_in[frame - num_points_outbound - dwell_frames]
        status = "Inbound: Mars to Earth"

    # Update the spacecraft marker position for 3D plot
    spacecraft.set_data([pos[0]], [pos[1]])
    spacecraft.set_3d_properties([pos[2]])

    # Update the status text
    status_text.set_text(f"Frame: {frame+1}/{total_frames} - {status}")

    return spacecraft, status_text


# Create the animation object (interval in milliseconds)
anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)

# Add a legend to identify plot elements
ax.legend()

plt.show()
