import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants (simplified distances in arbitrary units)
AU = 1.0  # Astronomical Unit
EARTH_ORBIT = 1.0 * AU
MARS_ORBIT = 1.524 * AU
TRANSFER_TIME = 100  # frames for transfer
ORBIT_TIME = 50  # frames for orbiting

# Create figure and 3D axes
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Set plot limits
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-2, 2)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars Round Trip Trajectory")

# Initialize orbital paths
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit_x = EARTH_ORBIT * np.cos(theta)
earth_orbit_y = EARTH_ORBIT * np.sin(theta)
mars_orbit_x = MARS_ORBIT * np.cos(theta)
mars_orbit_y = MARS_ORBIT * np.sin(theta)

# Plot static orbits
ax.plot(earth_orbit_x, earth_orbit_y, 0, "b-", label="Earth Orbit")
ax.plot(mars_orbit_x, mars_orbit_y, 0, "r-", label="Mars Orbit")
ax.plot([0], [0], [0], "yo", label="Sun")

# Initialize spacecraft position
(spacecraft,) = ax.plot([], [], [], "go", label="Spacecraft", markersize=8)

# Trajectory lines
(earth_to_mars_line,) = ax.plot([], [], [], "g--")
(mars_to_earth_line,) = ax.plot([], [], [], "g--")


def init():
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    earth_to_mars_line.set_data([], [])
    earth_to_mars_line.set_3d_properties([])
    mars_to_earth_line.set_data([], [])
    mars_to_earth_line.set_3d_properties([])
    return spacecraft, earth_to_mars_line, mars_to_earth_line


def update(frame):
    # Total animation frames
    total_frames = TRANSFER_TIME * 2 + ORBIT_TIME * 2

    # Earth to Mars transfer (first leg)
    if frame < TRANSFER_TIME:
        t = frame / TRANSFER_TIME
        x = np.linspace(EARTH_ORBIT, MARS_ORBIT, TRANSFER_TIME)[: frame + 1]
        y = np.linspace(0, 0, TRANSFER_TIME)[: frame + 1]  # Simplified straight path
        z = np.zeros(frame + 1)

        spacecraft.set_data([x[-1]], [y[-1]])
        spacecraft.set_3d_properties([z[-1]])
        earth_to_mars_line.set_data(x, y)
        earth_to_mars_line.set_3d_properties(z)

    # Mars orbit
    elif frame < TRANSFER_TIME + ORBIT_TIME:
        t = (frame - TRANSFER_TIME) / ORBIT_TIME * 2 * np.pi
        x = MARS_ORBIT * np.cos(t)
        y = MARS_ORBIT * np.sin(t)
        z = 0

        spacecraft.set_data([x], [y])
        spacecraft.set_3d_properties([z])

    # Mars to Earth transfer (return leg)
    elif frame < TRANSFER_TIME * 2 + ORBIT_TIME:
        t = (frame - (TRANSFER_TIME + ORBIT_TIME)) / TRANSFER_TIME
        x = np.linspace(MARS_ORBIT, EARTH_ORBIT, TRANSFER_TIME)[
            : int(t * TRANSFER_TIME) + 1
        ]
        y = np.linspace(0, 0, TRANSFER_TIME)[: int(t * TRANSFER_TIME) + 1]
        z = np.zeros(int(t * TRANSFER_TIME) + 1)

        spacecraft.set_data([x[-1]], [y[-1]])
        spacecraft.set_3d_properties([z[-1]])
        mars_to_earth_line.set_data(x, y)
        mars_to_earth_line.set_3d_properties(z)

    # Earth orbit
    else:
        t = (frame - (TRANSFER_TIME * 2 + ORBIT_TIME)) / ORBIT_TIME * 2 * np.pi
        x = EARTH_ORBIT * np.cos(t)
        y = EARTH_ORBIT * np.sin(t)
        z = 0

        spacecraft.set_data([x], [y])
        spacecraft.set_3d_properties([z])

    return spacecraft, earth_to_mars_line, mars_to_earth_line


# Create animation
anim = FuncAnimation(
    fig,
    update,
    frames=TRANSFER_TIME * 2 + ORBIT_TIME * 2,
    init_func=init,
    interval=50,
    blit=True,
)

ax.legend()
plt.show()
