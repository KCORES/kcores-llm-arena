import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.49598e11  # Astronomical unit in meters
earth_orbital_radius = 1.0  # In AU
mars_orbital_radius = 1.52  # In AU
earth_year = 365.25  # days
mars_year = 687  # days
transfer_time = 0.71  # Years for Hohmann transfer
wait_time = 1.22  # Years between transfer windows

# Create figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Set plot limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars-Earth Transfer Animation")

# Create orbital paths
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit = np.array(
    [
        earth_orbital_radius * np.cos(theta),
        earth_orbital_radius * np.sin(theta),
        np.zeros_like(theta),
    ]
)
mars_orbit = np.array(
    [
        mars_orbital_radius * np.cos(theta),
        mars_orbital_radius * np.sin(theta),
        0.1 * np.sin(2 * theta),
    ]
)

# Plot orbital paths
ax.plot(*earth_orbit, color="blue", alpha=0.3, label="Earth Orbit")
ax.plot(*mars_orbit, color="red", alpha=0.3, label="Mars Orbit")

# Create initial positions
earth = ax.plot([], [], [], "o", color="blue", markersize=10)[0]
mars = ax.plot([], [], [], "o", color="red", markersize=8)[0]
spacecraft = ax.plot([], [], [], "o", color="black", markersize=4)[0]
trajectory = ax.plot([], [], [], color="orange", alpha=0.5)[0]

# Initialize position lists
x_traj, y_traj, z_traj = [], [], []


def get_position(radius, angle, z=0):
    return (radius * np.cos(angle), radius * np.sin(angle), z)


def animate(i):
    time = i * 0.02  # Speed control

    # Calculate orbital positions
    earth_angle = 2 * np.pi * (time % 1)
    mars_angle = 2 * np.pi * (time % (mars_year / earth_year))

    # Update Earth position
    x_e, y_e, z_e = get_position(earth_orbital_radius, earth_angle)
    earth.set_data([x_e], [y_e])
    earth.set_3d_properties([z_e])

    # Update Mars position
    x_m, y_m, z_m = get_position(mars_orbital_radius, mars_angle)
    mars.set_data([x_m], [y_m])
    mars.set_3d_properties([0.1 * np.sin(2 * mars_angle)])

    # Calculate spacecraft position
    mission_phase = time % (transfer_time + wait_time + transfer_time)

    if mission_phase < transfer_time:
        phase = mission_phase / transfer_time
        x = (1 - phase) * earth_orbital_radius + phase * mars_orbital_radius
        y = 0.5 * np.sin(phase * np.pi)
        z = 0.05 * np.sin(2 * phase * np.pi)
        color = "orange"
    elif mission_phase < transfer_time + wait_time:
        x, y, z = x_m, y_m, z_m + 0.1 * np.sin(2 * mars_angle)
        color = "red"
    else:
        phase = (mission_phase - transfer_time - wait_time) / transfer_time
        x = (1 - phase) * mars_orbital_radius + phase * earth_orbital_radius
        y = 0.5 * np.sin((1 - phase) * np.pi)
        z = 0.05 * np.sin(2 * (1 - phase) * np.pi)
        color = "green"

    # Update spacecraft
    spacecraft.set_data([x], [y])
    spacecraft.set_3d_properties([z])
    spacecraft.set_color(color)

    # Update trajectory
    if len(x_traj) > 200:
        x_traj.pop(0)
        y_traj.pop(0)
        z_traj.pop(0)
    x_traj.append(x)
    y_traj.append(y)
    z_traj.append(z)

    trajectory.set_data(x_traj, y_traj)
    trajectory.set_3d_properties(z_traj)

    # Update date display
    year = 2033 + int(time)
    ax.text2D(
        0.05,
        0.95,
        f"Earth Date: {year}-{(time%1)*12+1:02.0f}-01",
        transform=ax.transAxes,
    )

    return earth, mars, spacecraft, trajectory


# Create animation
ani = animation.FuncAnimation(fig, animate, frames=500, interval=20, blit=True)

# Add legend and adjust view
ax.legend(loc="upper left")
ax.view_init(elev=30, azim=45)

plt.show()
