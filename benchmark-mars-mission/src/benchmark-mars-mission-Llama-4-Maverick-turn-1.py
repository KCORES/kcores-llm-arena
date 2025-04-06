import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
AU = 1.496e11  # Astronomical Unit in meters
DAY = 86400  # Day in seconds

# Planetary positions and velocities at J2000 epoch (simplified)
earth_position = np.array([AU, 0, 0])
earth_velocity = np.array([0, 29.78e3, 0])

mars_position = np.array([1.524 * AU, 0, 0])
mars_velocity = np.array([0, 24.07e3, 0])

# Spacecraft initial conditions
spacecraft_position = earth_position.copy()
spacecraft_velocity = earth_velocity.copy() + np.array(
    [0, 10e3, 0]
)  # delta-v for Mars transfer

# Time parameters
t_max = 780 * DAY  # Total mission duration (days)
dt = DAY / 10  # Time step (days)
t = np.arange(0, t_max, dt)

# Arrays to store positions and velocities
n_steps = len(t)
spacecraft_positions = np.zeros((n_steps, 3))
earth_positions = np.zeros((n_steps, 3))
mars_positions = np.zeros((n_steps, 3))

# Simplified 2-body problem simulation (Earth-Sun, Mars-Sun, Spacecraft-Sun)
for i in range(n_steps):
    # Update Earth and Mars positions (circular orbits)
    earth_angle = 2 * np.pi * t[i] / (365.25 * DAY)
    mars_angle = 2 * np.pi * t[i] / (687 * DAY)
    earth_positions[i] = AU * np.array([np.cos(earth_angle), np.sin(earth_angle), 0])
    mars_positions[i] = (
        1.524 * AU * np.array([np.cos(mars_angle), np.sin(mars_angle), 0])
    )

    # Update spacecraft position and velocity (simplified)
    if i == 0:
        spacecraft_positions[i] = spacecraft_position
    else:
        r_earth = spacecraft_positions[i - 1] - earth_positions[i - 1]
        r_mars = spacecraft_positions[i - 1] - mars_positions[i - 1]
        r_sun = spacecraft_positions[i - 1]
        a_earth = -6.67430e-11 * 5.97237e24 * r_earth / np.linalg.norm(r_earth) ** 3
        a_mars = -6.67430e-11 * 6.4171e23 * r_mars / np.linalg.norm(r_mars) ** 3
        a_sun = -6.67430e-11 * 1.989e30 * r_sun / np.linalg.norm(r_sun) ** 3
        spacecraft_velocity += (a_earth + a_mars + a_sun) * dt
        spacecraft_positions[i] = spacecraft_positions[i - 1] + spacecraft_velocity * dt

        # Simplified Mars orbit insertion and departure
        if np.linalg.norm(spacecraft_positions[i] - mars_positions[i]) < 1e7:
            spacecraft_velocity = mars_velocity + np.array(
                [0, 2e3, 0]
            )  # delta-v for Mars orbit
        if (
            np.linalg.norm(spacecraft_positions[i] - mars_positions[i]) < 1e7
            and t[i] > 400 * DAY
        ):
            spacecraft_velocity = mars_velocity + np.array(
                [0, -10e3, 0]
            )  # delta-v for Earth transfer

# 3D Animated Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


def animate(i):
    ax.clear()
    ax.set_xlim(-2 * AU, 2 * AU)
    ax.set_ylim(-2 * AU, 2 * AU)
    ax.set_zlim(-2 * AU, 2 * AU)
    ax.plot3D(
        earth_positions[:i, 0],
        earth_positions[:i, 1],
        earth_positions[:i, 2],
        "b",
        label="Earth",
    )
    ax.plot3D(
        mars_positions[:i, 0],
        mars_positions[:i, 1],
        mars_positions[:i, 2],
        "r",
        label="Mars",
    )
    ax.plot3D(
        spacecraft_positions[:i, 0],
        spacecraft_positions[:i, 1],
        spacecraft_positions[:i, 2],
        "g",
        label="Spacecraft",
    )
    ax.scatter(
        earth_positions[i, 0], earth_positions[i, 1], earth_positions[i, 2], c="b"
    )
    ax.scatter(mars_positions[i, 0], mars_positions[i, 1], mars_positions[i, 2], c="r")
    ax.scatter(
        spacecraft_positions[i, 0],
        spacecraft_positions[i, 1],
        spacecraft_positions[i, 2],
        c="g",
    )
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()


ani = animation.FuncAnimation(fig, animate, frames=len(t), interval=50)
plt.show()
