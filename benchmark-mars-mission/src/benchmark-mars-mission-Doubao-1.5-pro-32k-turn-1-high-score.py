import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


# Constants
AU = 1.496e11  # Astronomical unit (m)
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
M_sun = 1.989e30  # Mass of the Sun (kg)

# Orbital parameters
r_earth = 1.0 * AU
r_mars = 1.524 * AU

# Orbital periods
T_earth = 365.25 * 24 * 3600  # Orbital period of Earth (s)
T_mars = 687 * 24 * 3600  # Orbital period of Mars (s)

# Time parameters
t_max = 2 * T_mars  # Total simulation time
dt = 1 * 24 * 3600  # Time step (1 day)
t = np.arange(0, t_max, dt)

# Orbital positions of Earth and Mars
theta_earth = 2 * np.pi * t / T_earth
x_earth = r_earth * np.cos(theta_earth)
y_earth = r_earth * np.sin(theta_earth)
z_earth = np.zeros_like(x_earth)

theta_mars = 2 * np.pi * t / T_mars
x_mars = r_mars * np.cos(theta_mars)
y_mars = r_mars * np.sin(theta_mars)
z_mars = np.zeros_like(x_mars)

# Hohmann transfer orbit parameters
a_transfer = (r_earth + r_mars) / 2
T_transfer = 2 * np.pi * np.sqrt(a_transfer**3 / (G * M_sun))

# Find the launch window
phi_0 = np.arccos((r_earth + r_mars - 2 * a_transfer) / (r_mars))
phi_mars_launch = phi_0 + 2 * np.pi * (T_transfer / 2) / T_mars
launch_index = np.argmin(np.abs(theta_mars - phi_mars_launch))

# Forward Hohmann transfer
t_transfer_forward = np.arange(0, T_transfer / 2, dt)
theta_transfer_forward = 2 * np.pi * t_transfer_forward / T_transfer
x_transfer_forward = a_transfer * np.cos(theta_transfer_forward)
y_transfer_forward = a_transfer * np.sin(theta_transfer_forward)
z_transfer_forward = np.zeros_like(x_transfer_forward)

# Backward Hohmann transfer
t_transfer_backward = np.arange(T_transfer / 2, T_transfer, dt)
theta_transfer_backward = 2 * np.pi * t_transfer_backward / T_transfer
x_transfer_backward = a_transfer * np.cos(theta_transfer_backward)
y_transfer_backward = a_transfer * np.sin(theta_transfer_backward)
z_transfer_backward = np.zeros_like(x_transfer_backward)

# Combine transfer orbits
x_transfer = np.concatenate((x_transfer_forward, x_transfer_backward))
y_transfer = np.concatenate((y_transfer_forward, y_transfer_backward))
z_transfer = np.concatenate((z_transfer_forward, z_transfer_backward))

# Create the figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize the plots
sun_plot = ax.scatter(0, 0, 0, color='yellow', s=200, label='Sun')
earth_plot, = ax.plot([], [], [], color='blue', label='Earth')
mars_plot, = ax.plot([], [], [], color='red', label='Mars')
transfer_plot, = ax.plot([], [], [], color='green', label='Spacecraft')

# Set the axis limits
ax.set_xlim([-2 * r_mars, 2 * r_mars])
ax.set_ylim([-2 * r_mars, 2 * r_mars])
ax.set_zlim([-0.1 * r_mars, 0.1 * r_mars])

# Add legends
ax.legend()


# Animation update function
def update(frame):
    if frame < len(t):
        earth_plot.set_data(x_earth[:frame], y_earth[:frame])
        earth_plot.set_3d_properties(z_earth[:frame])
        mars_plot.set_data(x_mars[:frame], y_mars[:frame])
        mars_plot.set_3d_properties(z_mars[:frame])

        if frame >= launch_index and frame < launch_index + len(x_transfer):
            transfer_plot.set_data(x_transfer[:frame - launch_index], y_transfer[:frame - launch_index])
            transfer_plot.set_3d_properties(z_transfer[:frame - launch_index])
        else:
            transfer_plot.set_data([], [])
            transfer_plot.set_3d_properties([])

    return earth_plot, mars_plot, transfer_plot


# Create the animation
ani = FuncAnimation(fig, update, frames=len(t), interval=20, blit=True)

# Show the animation
plt.show()
