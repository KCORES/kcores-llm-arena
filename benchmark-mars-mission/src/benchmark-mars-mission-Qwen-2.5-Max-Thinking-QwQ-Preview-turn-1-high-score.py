import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.0  # 1 AU in display units
T_EARTH = 365  # Earth orbital period (days)
T_MARS = 687  # Mars orbital period (days)
T_TRANSFER = 259  # One-way transfer time (days)
A_TRANSFER = (1.0 + 1.524) / 2  # Transfer orbit semi-major axis (AU)
E_TRANSFER = (1.524 - 1.0) / (1.524 + 1.0)  # Transfer orbit eccentricity

# Mission timeline
WAIT_TIME = 212  # Days waiting on Mars
TOTAL_TIME = 2 * T_TRANSFER + WAIT_TIME  # Total mission duration (days)

# Time array for animation
dt = 1  # Day per frame
times = np.arange(0, TOTAL_TIME + dt, dt)

# Precompute planet positions
earth_angles = 2 * np.pi * times / T_EARTH
earth_x = np.cos(earth_angles)
earth_y = np.sin(earth_angles)

mars_angles = 2 * np.pi * times / T_MARS
mars_x = 1.524 * np.cos(mars_angles)
mars_y = 1.524 * np.sin(mars_angles)

# Precompute spacecraft trajectory
sc_x = []
sc_y = []
for t in times:
    if t <= T_TRANSFER:
        # Outbound transfer to Mars
        s = t / T_TRANSFER
        nu = np.pi * s
        r = A_TRANSFER * (1 - E_TRANSFER**2) / (1 + E_TRANSFER * np.cos(nu))
        x = r * np.cos(nu)
        y = r * np.sin(nu)
    elif t <= T_TRANSFER + WAIT_TIME:
        # Stationary on Mars
        idx = np.where(times == t)[0][0]
        x = mars_x[idx]
        y = mars_y[idx]
    else:
        # Return transfer to Earth
        s = (t - (T_TRANSFER + WAIT_TIME)) / T_TRANSFER
        nu = np.pi * (1 - s)
        r = A_TRANSFER * (1 - E_TRANSFER**2) / (1 + E_TRANSFER * np.cos(nu))
        x = r * np.cos(nu)
        y = r * np.sin(nu)
    sc_x.append(x)
    sc_y.append(y)

# Set up figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot orbital paths
earth_orbit = np.linspace(0, 2 * np.pi, 100)
ax.plot(np.cos(earth_orbit), np.sin(earth_orbit), 0, color="blue", alpha=0.3)
mars_orbit = np.linspace(0, 2 * np.pi, 100)
ax.plot(
    1.524 * np.cos(mars_orbit), 1.524 * np.sin(mars_orbit), 0, color="red", alpha=0.3
)

# Sun marker
ax.scatter([0], [0], [0], color="yellow", s=100)

# Initialize planet and spacecraft markers
(earth,) = ax.plot([], [], [], "o", color="blue", label="Earth")
(mars,) = ax.plot([], [], [], "o", color="red", label="Mars")
(spacecraft,) = ax.plot([], [], [], "o", color="green", label="Spacecraft")
(trajectory,) = ax.plot([], [], [], color="green", alpha=0.3)

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.legend()


def init():
    """Initialize animation"""
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
    """Update function for animation"""
    # Update planet positions
    earth.set_data([earth_x[frame]], [earth_y[frame]])
    earth.set_3d_properties([0])
    mars.set_data([mars_x[frame]], [mars_y[frame]])
    mars.set_3d_properties([0])

    # Update spacecraft position
    spacecraft.set_data([sc_x[frame]], [sc_y[frame]])
    spacecraft.set_3d_properties([0])

    # Update trajectory line
    trajectory.set_data(sc_x[: frame + 1], sc_y[: frame + 1])
    trajectory.set_3d_properties(np.zeros(frame + 1))

    return earth, mars, spacecraft, trajectory


# Create animation
ani = FuncAnimation(
    fig, update, frames=len(times), init_func=init, blit=True, interval=50
)

plt.show()
