import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Set up the figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Basic parameters (not to scale, for visualization)
earth_orbit = 1.0  # AU
mars_orbit = 1.524  # AU
time_steps = 200
synodic_period = 780  # days, approximate Earth-Mars synodic period

# Create orbital paths
t = np.linspace(0, 4 * np.pi, time_steps)
earth_x = earth_orbit * np.cos(t)
earth_y = earth_orbit * np.sin(t)
earth_z = np.zeros(time_steps)

mars_t = t * 0.53  # Mars moves slower (approx orbital period ratio)
mars_x = mars_orbit * np.cos(mars_t)
mars_y = mars_orbit * np.sin(mars_t)
mars_z = np.zeros(time_steps)


# Simple transfer trajectories (not real Hohmann transfers)
def transfer_trajectory(start_pos, end_pos, t, outbound=True):
    t_norm = t / (time_steps - 1)
    if outbound:
        radius = earth_orbit + (mars_orbit - earth_orbit) * t_norm
    else:
        radius = mars_orbit - (mars_orbit - earth_orbit) * t_norm
    angle = t_norm * np.pi
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = t_norm * 0.2 * np.sin(angle)  # Small z-component for 3D effect
    return x, y, z


# Initialize plot elements
(earth_line,) = ax.plot([], [], [], "b-", label="Earth orbit")
(mars_line,) = ax.plot([], [], [], "r-", label="Mars orbit")
(transfer_line,) = ax.plot([], [], [], "g-", label="Transfer trajectory")
(earth_point,) = ax.plot([], [], [], "bo", markersize=10, label="Earth")
(mars_point,) = ax.plot([], [], [], "ro", markersize=8, label="Mars")
(ship_point,) = ax.plot([], [], [], "g^", markersize=8, label="Spacecraft")

# Set plot limits and labels
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars-Earth Transfer Animation")
ax.legend()


def init():
    earth_line.set_data([], [])
    earth_line.set_3d_properties([])
    mars_line.set_data([], [])
    mars_line.set_3d_properties([])
    transfer_line.set_data([], [])
    transfer_line.set_3d_properties([])
    earth_point.set_data([], [])
    earth_point.set_3d_properties([])
    mars_point.set_data([], [])
    mars_point.set_3d_properties([])
    ship_point.set_data([], [])
    ship_point.set_3d_properties([])
    return (earth_line, mars_line, transfer_line, earth_point, mars_point, ship_point)


def animate(frame):
    # Full animation cycle: Earth -> Mars -> wait -> Mars -> Earth
    total_frames = 600
    phase = frame % total_frames

    # Plot static orbital paths
    earth_line.set_data(earth_x, earth_y)
    earth_line.set_3d_properties(earth_z)
    mars_line.set_data(mars_x, mars_y)
    mars_line.set_3d_properties(mars_z)

    # Update planet positions
    earth_idx = frame % time_steps
    mars_idx = int((frame * 0.53) % time_steps)
    earth_point.set_data([earth_x[earth_idx]], [earth_y[earth_idx]])
    earth_point.set_3d_properties([earth_z[earth_idx]])
    mars_point.set_data([mars_x[mars_idx]], [mars_y[mars_idx]])
    mars_point.set_3d_properties([mars_z[mars_idx]])

    # Transfer phases
    if phase < 200:  # Earth to Mars (0-199)
        t = np.linspace(0, phase, phase + 1)
        x, y, z = transfer_trajectory(
            [earth_x[0], earth_y[0]], [mars_x[0], mars_y[0]], t, outbound=True
        )
        transfer_line.set_data(x, y)
        transfer_line.set_3d_properties(z)
        ship_point.set_data([x[-1]], [y[-1]])
        ship_point.set_3d_properties([z[-1]])

    elif phase < 400:  # Waiting at Mars (200-399)
        transfer_line.set_data([], [])
        transfer_line.set_3d_properties([])
        ship_point.set_data([mars_x[mars_idx]], [mars_y[mars_idx]])
        ship_point.set_3d_properties([mars_z[mars_idx]])

    else:  # Mars to Earth (400-599)
        t = np.linspace(0, phase - 400, phase - 399)
        x, y, z = transfer_trajectory(
            [mars_x[0], mars_y[0]], [earth_x[0], earth_y[0]], t, outbound=False
        )
        transfer_line.set_data(x, y)
        transfer_line.set_3d_properties(z)
        ship_point.set_data([x[-1]], [y[-1]])
        ship_point.set_3d_properties([z[-1]])

    return (earth_line, mars_line, transfer_line, earth_point, mars_point, ship_point)


# Create animation
ani = animation.FuncAnimation(
    fig, animate, init_func=init, frames=600, interval=20, blit=True
)

plt.show()
