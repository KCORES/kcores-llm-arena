import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.animation as animation

# --- Simulation Parameters (Simplified) ---
earth_radius = 6371  # km
mars_radius = 3389.5  # km
earth_orbit_radius = 149.6e6  # km (1 AU)
mars_orbit_radius = 227.9e6  # km (1.524 AU)
earth_orbital_period = 365.25  # days
mars_orbital_period = 687  # days

# Approximate launch windows and travel times (very simplified)
earth_mars_transfer_duration = 250  # days (approximate)
mars_earth_transfer_duration = 250  # days (approximate)
mars_stay_duration = 500  # days (approximate) - to wait for next launch window

# Time steps for animation
time_step = 5  # days per frame (adjust for animation speed)
total_simulation_time = (
    earth_orbital_period
    + earth_mars_transfer_duration
    + mars_stay_duration
    + mars_earth_transfer_duration
    + 100
)  # Extra time for visualization

time = np.arange(0, total_simulation_time, time_step)


# --- Planetary Positions (Simplified Circular Orbits) ---
def planetary_position(radius, period, time, phase_offset=0):
    """Calculates planetary position in 2D (x, y) assuming circular orbit in x-y plane.
    Ignores orbital inclination and eccentricity for simplicity."""
    angle = (2 * np.pi * time / period) + phase_offset
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0  # Assume orbits are in x-y plane for simplicity
    return x, y, z


earth_positions = np.array(
    [planetary_position(earth_orbit_radius, earth_orbital_period, t) for t in time]
).T
mars_positions = np.array(
    [
        planetary_position(mars_orbit_radius, mars_orbital_period, t, phase_offset=1.5)
        for t in time
    ]
).T  # Added phase offset for initial separation

# --- Spacecraft Trajectory (Simplified - Linear Interpolation for demonstration) ---
launch_time_index = 0  # Start at time 0
mars_arrival_time_index = int(earth_mars_transfer_duration / time_step)
mars_departure_time_index = mars_arrival_time_index + int(
    mars_stay_duration / time_step
)
earth_return_time_index = mars_departure_time_index + int(
    mars_earth_transfer_duration / time_step
)

# Earth to Mars trajectory (linear interpolation between Earth at launch and Mars at arrival)
traj_earth_mars_x = np.linspace(
    earth_positions[0, launch_time_index],
    mars_positions[0, mars_arrival_time_index],
    mars_arrival_time_index - launch_time_index + 1,
)
traj_earth_mars_y = np.linspace(
    earth_positions[1, launch_time_index],
    mars_positions[1, mars_arrival_time_index],
    mars_arrival_time_index - launch_time_index + 1,
)
traj_earth_mars_z = np.linspace(
    earth_positions[2, launch_time_index],
    mars_positions[2, mars_arrival_time_index],
    mars_arrival_time_index - launch_time_index + 1,
)

# Mars to Earth trajectory (linear interpolation between Mars at departure and Earth at return)
traj_mars_earth_x = np.linspace(
    mars_positions[0, mars_departure_time_index],
    earth_positions[0, earth_return_time_index],
    earth_return_time_index - mars_departure_time_index + 1,
)
traj_mars_earth_y = np.linspace(
    mars_positions[1, mars_departure_time_index],
    earth_positions[1, earth_return_time_index],
    earth_return_time_index - mars_departure_time_index + 1,
)
traj_mars_earth_z = np.linspace(
    mars_positions[2, mars_departure_time_index],
    earth_positions[2, earth_return_time_index],
    earth_return_time_index - mars_departure_time_index + 1,
)

# Combine trajectories
spacecraft_traj_x = np.concatenate([traj_earth_mars_x, traj_mars_earth_x])
spacecraft_traj_y = np.concatenate([traj_earth_mars_y, traj_mars_earth_y])
spacecraft_traj_z = np.concatenate([traj_earth_mars_z, traj_mars_earth_z])


# --- Visualization ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Planet spheres (simplified - just points for animation efficiency)
(earth_point,) = ax.plot(
    [earth_positions[0, 0]],
    [earth_positions[1, 0]],
    [earth_positions[2, 0]],
    "o",
    markersize=10,
    color="blue",
    label="Earth",
)
(mars_point,) = ax.plot(
    [mars_positions[0, 0]],
    [mars_positions[1, 0]],
    [mars_positions[2, 0]],
    "o",
    markersize=8,
    color="red",
    label="Mars",
)

# Spacecraft trajectory line
(traj_line,) = ax.plot(
    [], [], [], "-", color="gray", linewidth=2, label="Spacecraft Trajectory"
)
(spacecraft_point,) = ax.plot(
    [spacecraft_traj_x[0]],
    [spacecraft_traj_y[0]],
    [spacecraft_traj_z[0]],
    marker="o",
    markersize=4,
    color="green",
    label="Spacecraft",
)


# Set plot limits to encompass orbits (adjust scale as needed)
max_orbit = max(earth_orbit_radius, mars_orbit_radius) * 1.2
ax.set_xlim([-max_orbit, max_orbit])
ax.set_ylim([-max_orbit, max_orbit])
ax.set_zlim(
    [-max_orbit * 0.2, max_orbit * 0.2]
)  # Z-axis smaller for better view of x-y plane orbits

ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Mission (Simplified Animation)")
ax.legend(loc="upper right")
ax.set_aspect("equal")  # Ensure aspect ratio is equal for accurate 3D representation
ax.view_init(elev=30, azim=-45)  # Initial camera view


# --- Animation Function ---
def animate(i):
    frame_time_index = i % len(time)  # Loop animation if needed

    # Update planet positions
    earth_point.set_data(
        [earth_positions[0, frame_time_index]], [earth_positions[1, frame_time_index]]
    )
    earth_point.set_3d_properties([earth_positions[2, frame_time_index]])
    mars_point.set_data(
        [mars_positions[0, frame_time_index]], [mars_positions[1, frame_time_index]]
    )
    mars_point.set_3d_properties([mars_positions[2, frame_time_index]])

    # Update spacecraft trajectory and position
    if frame_time_index <= len(spacecraft_traj_x) - 1:
        traj_line.set_data(
            spacecraft_traj_x[: frame_time_index + 1],
            spacecraft_traj_y[: frame_time_index + 1],
        )
        traj_line.set_3d_properties(spacecraft_traj_z[: frame_time_index + 1])
        spacecraft_point.set_data(
            [spacecraft_traj_x[frame_time_index]], [spacecraft_traj_y[frame_time_index]]
        )
        spacecraft_point.set_3d_properties([spacecraft_traj_z[frame_time_index]])
    else:
        traj_line.set_data(
            spacecraft_traj_x, spacecraft_traj_y
        )  # Show full trajectory after it's complete
        traj_line.set_3d_properties(spacecraft_traj_z)
        spacecraft_point.set_data(
            [spacecraft_traj_x[-1]], [spacecraft_traj_y[-1]]
        )  # Keep spacecraft at end
        spacecraft_point.set_3d_properties([spacecraft_traj_z[-1]])

    return (
        earth_point,
        mars_point,
        traj_line,
        spacecraft_point,
    )


# Create animation
ani = animation.FuncAnimation(
    fig, animate, frames=len(time), interval=50, blit=False
)  # Adjust interval for speed

plt.show()

# To save the animation as a GIF (requires Pillow and imageio-ffmpeg)
# ani.save('earth_mars_earth_mission.gif', writer='imagemagick', fps=30) # Or writer='ffmpeg'
