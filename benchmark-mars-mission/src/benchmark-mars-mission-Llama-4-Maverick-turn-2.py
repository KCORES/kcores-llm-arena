import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
AU = 1.496e11  # Astronomical Unit in meters
DAY = 86400  # Day in seconds

# Simplified orbital parameters (circular orbits assumed)
r_earth = 1 * AU  # Average distance of Earth from the Sun
r_mars = 1.524 * AU  # Average distance of Mars from the Sun

# Orbital periods (simplified, actual periods are slightly different)
T_earth = 365.25 * DAY
T_mars = 687.0 * DAY

# Angular velocities
omega_earth = 2 * np.pi / T_earth
omega_mars = 2 * np.pi / T_mars

# Launch window and travel time simplifications
launch_window = 780 * DAY  # Approximate synodic period between Earth and Mars
travel_time = 200 * DAY  # Simplified travel time from Earth to Mars

# Initial phase angles (radians)
theta_earth_0 = 0
theta_mars_0 = (
    np.pi
)  # Mars is on the opposite side initially for a simple transfer orbit

# Generate time array
t = np.linspace(0, 2 * launch_window + travel_time, 500)

# Positions of Earth and Mars over time
theta_earth = omega_earth * t + theta_earth_0
theta_mars = omega_mars * t + theta_mars_0

x_earth = r_earth * np.cos(theta_earth)
y_earth = r_earth * np.sin(theta_earth)
z_earth = np.zeros_like(t)

x_mars = r_mars * np.cos(theta_mars)
y_mars = r_mars * np.sin(theta_mars)
z_mars = np.zeros_like(t)

# Simple spacecraft trajectory (Earth to Mars and back)
x_sc = np.zeros_like(t)
y_sc = np.zeros_like(t)
z_sc = np.zeros_like(t)

for i, ti in enumerate(t):
    if ti < travel_time:  # Traveling from Earth to Mars
        frac = ti / travel_time
        x_sc[i] = (1 - frac) * x_earth[0] + frac * x_mars[i]
        y_sc[i] = (1 - frac) * y_earth[0] + frac * y_mars[i]
    elif ti < launch_window:  # Staying on Mars
        x_sc[i] = x_mars[i]
        y_sc[i] = y_mars[i]
    elif ti < launch_window + travel_time:  # Traveling back to Earth
        frac = (ti - launch_window) / travel_time
        x_sc[i] = (1 - frac) * x_mars[i] + frac * x_earth[0]
        y_sc[i] = (1 - frac) * y_mars[i] + frac * y_earth[0]
    else:  # Orbiting Earth
        x_sc[i] = x_earth[i]
        y_sc[i] = y_earth[i]

# Create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")


# Animation function
def animate(i):
    ax.clear()
    ax.set_xlim(-2 * AU, 2 * AU)
    ax.set_ylim(-2 * AU, 2 * AU)
    ax.set_zlim(-2 * AU, 2 * AU)

    ax.plot(x_earth[:i], y_earth[:i], z_earth[:i], "b", label="Earth")
    ax.plot(x_mars[:i], y_mars[:i], z_mars[:i], "r", label="Mars")
    ax.plot(x_sc[:i], y_sc[:i], z_sc[:i], "g", label="Spacecraft")

    ax.scatter(x_earth[i], y_earth[i], z_earth[i], c="b")
    ax.scatter(x_mars[i], y_mars[i], z_mars[i], c="r")
    ax.scatter(x_sc[i], y_sc[i], z_sc[i], c="g")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend()


ani = animation.FuncAnimation(fig, animate, frames=len(t), interval=50)

plt.show()
