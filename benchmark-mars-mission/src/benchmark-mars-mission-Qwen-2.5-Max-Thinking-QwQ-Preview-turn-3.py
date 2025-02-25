import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.0  # Astronomical Unit in scaled coordinates
EARTH_ORBIT = 1.0
MARS_ORBIT = 1.5
EARTH_PERIOD = 365  # days
MARS_PERIOD = 687  # days
TRANSFER_TIME = 255  # days for Hohmann transfer
WAIT_TIME = 525  # days on Mars for return window
TOTAL_TIME = TRANSFER_TIME * 2 + WAIT_TIME

# Hohmann transfer parameters
a_transfer = (EARTH_ORBIT + MARS_ORBIT) / 2
e_transfer = (MARS_ORBIT - EARTH_ORBIT) / (MARS_ORBIT + EARTH_ORBIT)

# Mars initial phase calculation
mars_initial_angle = np.deg2rad(180 - (360 / MARS_PERIOD) * TRANSFER_TIME)

# Set up figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-1, 1])
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_title("Earth-Mars Round Trip Trajectory")

# Draw orbits
earth_orbit = np.linspace(0, 2 * np.pi, 100)
mars_orbit = np.linspace(0, 2 * np.pi, 100)
ax.plot(
    EARTH_ORBIT * np.cos(earth_orbit),
    EARTH_ORBIT * np.sin(earth_orbit),
    0,
    color="blue",
    alpha=0.3,
)
ax.plot(
    MARS_ORBIT * np.cos(mars_orbit),
    MARS_ORBIT * np.sin(mars_orbit),
    0,
    color="red",
    alpha=0.3,
)

# Initialize objects
(earth,) = ax.plot([], [], "o", color="blue", label="Earth")
(mars,) = ax.plot([], [], "o", color="red", label="Mars")
(spacecraft,) = ax.plot([], [], "o", color="green", label="Spacecraft")
(path,) = ax.plot([], [], color="green", alpha=0.3)
trajectory = {"x": [], "y": [], "z": []}


def init():
    earth.set_data([], [])
    mars.set_data([], [])
    spacecraft.set_data([], [])
    path.set_data([], [])
    return earth, mars, spacecraft, path


def animate(t):
    # Time in days
    current_time = t

    # Earth's position
    earth_angle = 2 * np.pi * current_time / EARTH_PERIOD
    ex = EARTH_ORBIT * np.cos(earth_angle)
    ey = EARTH_ORBIT * np.sin(earth_angle)

    # Mars' position
    mars_angle = 2 * np.pi * current_time / MARS_PERIOD + mars_initial_angle
    mx = MARS_ORBIT * np.cos(mars_angle)
    my = MARS_ORBIT * np.sin(mars_angle)

    # Spacecraft position
    if current_time < TRANSFER_TIME:
        # Outbound transfer
        s = current_time / TRANSFER_TIME
        nu = np.pi * s
        r = (a_transfer * (1 - e_transfer**2)) / (1 + e_transfer * np.cos(nu))
        sx = r * np.cos(nu)
        sy = r * np.sin(nu)
    elif current_time < TRANSFER_TIME + WAIT_TIME:
        # On Mars
        sx, sy = mx, my
    elif current_time < TOTAL_TIME:
        # Return transfer
        s = (current_time - TRANSFER_TIME - WAIT_TIME) / TRANSFER_TIME
        nu = np.pi * (1 - s)
        r = (a_transfer * (1 - e_transfer**2)) / (1 + e_transfer * np.cos(nu))
        sx = r * np.cos(nu)
        sy = r * np.sin(nu)
    else:
        # Back to Earth
        sx, sy = ex, ey

    # Update trajectory
    trajectory["x"].append(sx)
    trajectory["y"].append(sy)
    trajectory["z"].append(0)

    # Update plot elements
    earth.set_data([ex], [ey])
    mars.set_data([mx], [my])
    spacecraft.set_data([sx], [sy])
    path.set_data(trajectory["x"], trajectory["y"])

    return earth, mars, spacecraft, path


# Create animation
ani = FuncAnimation(
    fig,
    animate,
    frames=np.arange(0, TOTAL_TIME, 1),
    init_func=init,
    blit=True,
    interval=20,
)

plt.legend()
plt.tight_layout()
plt.show()
