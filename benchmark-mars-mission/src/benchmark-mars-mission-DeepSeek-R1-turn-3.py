import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.49598e8  # Astronomical Unit in km
earth_orbital_radius = 1.0  # in AU
mars_orbital_radius = 1.524  # in AU
earth_orbital_period = 365.25  # days
mars_orbital_period = 687  # days
transfer_time = 258  # days for Hohmann transfer

# Set up the figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("black")

# Set plot limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars-Earth Trajectory Animation")

# Create initial positions for celestial bodies
sun = ax.scatter([0], [0], [0], color="yellow", s=100, label="Sun")
(earth,) = ax.plot([], [], [], "bo", markersize=8, label="Earth")
(mars,) = ax.plot([], [], [], "ro", markersize=6, label="Mars")
(spacecraft,) = ax.plot([], [], [], "w*", markersize=10, label="Spacecraft")
(trajectory,) = ax.plot([], [], [], "y--", linewidth=0.5)

# Orbital angles
earth_angle = [0]
mars_angle = [0]
transfer_angle = [0]

# Mission phases
phase = "outbound"
current_day = [0]
wait_time = 500  # days to wait on Mars for return window


def init():
    earth.set_data([], [])
    earth.set_3d_properties([])
    mars.set_data([], [])
    mars.set_3d_properties([])
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    trajectory.set_data([], [])
    trajectory.set_3d_properties([])
    return earth, mars, spacecraft, trajectory


def update(frame):
    global phase, current_day

    current_day[0] += 5  # Days per frame

    # Update Earth position
    earth_theta = 2 * np.pi * (current_day[0] / earth_orbital_period)
    earth_x = earth_orbital_radius * np.cos(earth_theta)
    earth_y = earth_orbital_radius * np.sin(earth_theta)
    earth.set_data([earth_x], [earth_y])
    earth.set_3d_properties([0])

    # Update Mars position
    mars_theta = 2 * np.pi * (current_day[0] / mars_orbital_period)
    mars_x = mars_orbital_radius * np.cos(mars_theta)
    mars_y = mars_orbital_radius * np.sin(mars_theta)
    mars.set_data([mars_x], [mars_y])
    mars.set_3d_properties([0])

    # Update spacecraft position
    if phase == "outbound":
        transfer_progress = current_day[0] / transfer_time
        if transfer_progress >= 1:
            phase = "waiting"
            current_day[0] = transfer_time
        else:
            # Hohmann transfer orbit
            theta = np.pi * transfer_progress
            r = (earth_orbital_radius + mars_orbital_radius) / 2 * (1 - np.cos(theta))
            x = r * np.cos(theta + earth_theta)
            y = r * np.sin(theta + earth_theta)
            spacecraft.set_data([x], [y])
            spacecraft.set_3d_properties([0])

    elif phase == "waiting":
        # Stay with Mars
        spacecraft.set_data([mars_x], [mars_y])
        spacecraft.set_3d_properties([0])
        if current_day[0] > transfer_time + wait_time:
            phase = "return"
            current_day[0] = transfer_time + wait_time

    elif phase == "return":
        transfer_progress = (current_day[0] - transfer_time - wait_time) / transfer_time
        if transfer_progress >= 1:
            current_day[0] = 0
            phase = "outbound"
        else:
            # Return transfer orbit
            theta = np.pi * transfer_progress
            r = (earth_orbital_radius + mars_orbital_radius) / 2 * (1 - np.cos(theta))
            x = r * np.cos(theta + mars_theta)
            y = r * np.sin(theta + mars_theta)
            spacecraft.set_data([x], [y])
            spacecraft.set_3d_properties([0])

    # Update trajectory
    old_data = trajectory.get_data_3d()
    new_x = list(old_data[0]) + [spacecraft.get_data_3d()[0][0]]
    new_y = list(old_data[1]) + [spacecraft.get_data_3d()[1][0]]
    new_z = list(old_data[2]) + [spacecraft.get_data_3d()[2][0]]
    trajectory.set_data(new_x, new_y)
    trajectory.set_3d_properties(new_z)

    # Adjust view angle
    ax.view_init(elev=30, azim=current_day[0] / 5)

    return earth, mars, spacecraft, trajectory


# Create animation
ani = animation.FuncAnimation(
    fig, update, frames=200, init_func=init, blit=True, interval=50
)

# Add legend and show
ax.legend(loc="upper left")
plt.show()
