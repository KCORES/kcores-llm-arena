import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 149.6e6  # Astronomical Unit in km
year = 365.25  # Earth year in days
day = 86400  # Day in seconds
G = 6.67430e-11  # Gravitational constant
M_sun = 1.989e30  # Mass of the sun in kg

# Orbital parameters
r_earth = 1.0 * AU  # Earth's orbital radius
r_mars = 1.524 * AU  # Mars' orbital radius
T_earth = year * day  # Earth's orbital period
T_mars = 1.88 * year * day  # Mars' orbital period

# Velocity calculations
v_earth = 2 * np.pi * r_earth / T_earth
v_mars = 2 * np.pi * r_mars / T_mars

# Hohmann transfer orbit
r_transfer = (r_earth + r_mars) / 2
T_transfer = np.pi * np.sqrt(r_transfer**3 / (G * M_sun))
v_transfer = np.sqrt(G * M_sun * (2 / r_earth - 1 / r_transfer)) - v_earth

# Time parameters
t_max = 2 * T_transfer
dt = T_transfer / 100
times = np.arange(0, t_max, dt)


# Positions
def get_positions(t):
    theta_earth = 2 * np.pi * t / T_earth
    earth_pos = r_earth * np.array([np.cos(theta_earth), np.sin(theta_earth), 0])

    theta_mars = 2 * np.pi * t / T_mars
    mars_pos = r_mars * np.array([np.cos(theta_mars), np.sin(theta_mars), 0])

    if t < T_transfer:
        theta_transfer = np.pi * t / T_transfer
        transfer_pos = r_transfer * np.array(
            [np.cos(theta_transfer), np.sin(theta_transfer), 0]
        )
    else:
        theta_transfer = np.pi * (t - T_transfer) / T_transfer
        transfer_pos = r_transfer * np.array(
            [-np.cos(theta_transfer), -np.sin(theta_transfer), 0]
        )

    return earth_pos, mars_pos, transfer_pos


# Animation
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


def update(frame):
    ax.clear()

    # Plot Earth and Mars
    earth_pos, mars_pos, transfer_pos = get_positions(frame * dt)
    ax.scatter(*earth_pos, color="blue", label="Earth")
    ax.scatter(*mars_pos, color="red", label="Mars")

    # Plot transfer orbit
    ax.plot(
        [earth_pos[0], transfer_pos[0]],
        [earth_pos[1], transfer_pos[1]],
        [earth_pos[2], transfer_pos[2]],
        color="green",
        label="Transfer Orbit",
    )

    # Plot spacecraft
    ax.scatter(*transfer_pos, color="purple", label="Spacecraft")

    # Set limits and labels
    ax.set_xlim([-r_mars * 1.2, r_mars * 1.2])
    ax.set_ylim([-r_mars * 1.2, r_mars * 1.2])
    ax.set_zlim([-r_mars * 0.2, r_mars * 0.2])
    ax.set_xlabel("X [AU]")
    ax.set_ylabel("Y [AU]")
    ax.set_zlabel("Z [AU]")
    ax.legend()


ani = FuncAnimation(fig, update, frames=len(times), interval=50, blit=False)

plt.show()
