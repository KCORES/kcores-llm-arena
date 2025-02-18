import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# --- Planetary Parameters (Simplified) ---
earth_radius = 6371  # km
mars_radius = 3389.5  # km
earth_orbit_radius = 149.6e6  # km (1 AU)
mars_orbit_radius = 227.9e6  # km (1.52 AU)
earth_orbital_period = 365.25  # days
mars_orbital_period = 687  # days

# --- Mission Parameters (Simplified - adjust for realistic values) ---
earth_mars_transfer_duration = 250  # days (approximate Hohmann transfer)
mars_stay_duration = 450  # days (example - needs to be adjusted for launch windows)
mars_earth_transfer_duration = 250  # days (approximate Hohmann transfer)

total_mission_duration = (
    earth_mars_transfer_duration + mars_stay_duration + mars_earth_transfer_duration
)

# --- Time Setup for Animation ---
num_frames = 500  # Number of frames in the animation
time_points = np.linspace(0, total_mission_duration, num_frames)
frame_interval = total_mission_duration / num_frames  # days per frame


# --- Helper Functions for Planet Positions (Circular Orbits - Simplified) ---
def earth_position(time_days):
    angle = (time_days / earth_orbital_period) * 2 * np.pi  # Angle in radians
    x = earth_orbit_radius * np.cos(angle)
    y = earth_orbit_radius * np.sin(angle)
    z = 0  # Assume orbits are roughly in the xy-plane for simplicity
    return np.array([x, y, z])


def mars_position(time_days):
    angle = (time_days / mars_orbital_period) * 2 * np.pi
    x = mars_orbit_radius * np.cos(angle)
    y = mars_orbit_radius * np.sin(angle)
    z = 0
    return np.array([x, y, z])


# --- Trajectory Path Generation (Linear Interpolation - Highly Simplified) ---
def generate_trajectory():
    trajectory_points = []

    # Earth to Mars Transfer
    start_earth_pos = earth_position(0)
    end_mars_pos = mars_position(earth_mars_transfer_duration)
    for t in np.linspace(
        0,
        earth_mars_transfer_duration,
        int(num_frames * (earth_mars_transfer_duration / total_mission_duration)),
    ):  # Proportional frames
        progress = t / earth_mars_transfer_duration
        trajectory_pos = start_earth_pos + progress * (end_mars_pos - start_earth_pos)
        trajectory_points.append(trajectory_pos)

    # Mars Stay (Stationary on Mars surface for simplicity - could be orbiting Mars)
    mars_landing_pos = end_mars_pos  # Final position from Earth-Mars transfer
    for _ in np.linspace(
        0,
        mars_stay_duration,
        int(num_frames * (mars_stay_duration / total_mission_duration)),
    ):  # Proportional frames
        trajectory_points.append(mars_landing_pos)  # Stay at Mars landing location

    # Mars to Earth Transfer
    start_mars_pos_return = mars_position(
        earth_mars_transfer_duration + mars_stay_duration
    )  # Mars position at return launch window
    end_earth_pos_return = earth_position(
        total_mission_duration
    )  # Earth position at return arrival
    for t in np.linspace(
        0,
        mars_earth_transfer_duration,
        int(num_frames * (mars_earth_transfer_duration / total_mission_duration)),
    ):  # Proportional frames
        progress = t / mars_earth_transfer_duration
        trajectory_pos = start_mars_pos_return + progress * (
            end_earth_pos_return - start_mars_pos_return
        )
        trajectory_points.append(trajectory_pos)

    return np.array(trajectory_points)


trajectory_data = generate_trajectory()


# --- 3D Plot and Animation Setup ---
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(projection="3d")

# --- Planet Spheres ---
u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
earth_x = earth_radius * np.cos(u) * np.sin(v)
earth_y = earth_radius * np.sin(u) * np.sin(v)
earth_z = earth_radius * np.cos(v)
mars_x = mars_radius * np.cos(u) * np.sin(v)
mars_y = mars_radius * np.sin(u) * np.sin(v)
mars_z = mars_radius * np.cos(v)

earth_surface = ax.plot_surface(earth_x, earth_y, earth_z, color="blue", alpha=0.6)
mars_surface = ax.plot_surface(mars_x, mars_y, mars_z, color="red", alpha=0.6)

# --- Trajectory Lines ---
(trajectory_line,) = ax.plot([], [], [], color="gray", linewidth=2)
(spacecraft_point,) = ax.plot([], [], [], marker="o", markersize=8, color="green")


# --- Animation Function ---
def animate(frame):
    current_time = time_points[frame]

    # Update planet positions (static for simplicity in this example, could be animated)
    earth_pos = earth_position(0)  # Earth at origin for relative motion visualization
    mars_pos = mars_position(current_time)

    earth_surface._offsets3d = (
        earth_x + earth_pos[0],
        earth_y + earth_pos[1],
        earth_z + earth_pos[2],
    )  # Keep Earth at origin
    mars_surface._offsets3d = (
        mars_x + mars_pos[0],
        mars_y + mars_pos[1],
        mars_z + mars_pos[2],
    )

    # Update trajectory and spacecraft
    trajectory_line.set_data(trajectory_data[:frame, 0], trajectory_data[:frame, 1])
    trajectory_line.set_3d_properties(trajectory_data[:frame, 2])
    spacecraft_point.set_data(trajectory_data[frame, 0], trajectory_data[frame, 1])
    spacecraft_point.set_3d_properties(trajectory_data[frame, 2])

    ax.set_title(f"Mission Time: {current_time:.1f} days")
    return earth_surface, mars_surface, trajectory_line, spacecraft_point


# --- Animation Settings ---
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_xlim([-2.5e8, 2.5e8])  # Adjust limits to encompass orbits and trajectory
ax.set_ylim([-2.5e8, 2.5e8])
ax.set_zlim([-1e8, 1e8])  # Z limits a bit smaller
ax.view_init(elev=20, azim=-45)  # Initial view angle
ax.set_aspect("equal")  # Ensure equal aspect ratio for accurate 3D visualization

ani = FuncAnimation(
    fig, animate, frames=num_frames, interval=50, blit=False
)  # interval in milliseconds

plt.tight_layout()
plt.show()

# To save the animation (optional):
# ani.save('earth_mars_earth_mission.gif', writer='pillow') # or 'ffmpeg'
