import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


# Constants
AU = 1.496e11  # Astronomical unit in meters
G = 6.67430e-11  # Gravitational constant in m^3 kg^-1 s^-2
M_sun = 1.989e30  # Mass of the Sun in kg

# Orbital parameters (simplified circular orbits)
r_earth = 1.0 * AU
r_mars = 1.52 * AU

# Time parameters
t_total = 10 * 365 * 24 * 3600  # 10 years in seconds
dt = 3600  # Time step in seconds
t = np.arange(0, t_total, dt)

# Orbital positions of Earth and Mars (simplified circular orbits)
omega_earth = np.sqrt(G * M_sun / r_earth**3)
omega_mars = np.sqrt(G * M_sun / r_mars**3)

x_earth = r_earth * np.cos(omega_earth * t)
y_earth = r_earth * np.sin(omega_earth * t)
z_earth = np.zeros_like(t)

x_mars = r_mars * np.cos(omega_mars * t)
y_mars = r_mars * np.sin(omega_mars * t)
z_mars = np.zeros_like(t)


# Simplified mission trajectory
# Assume a simple linear transfer for illustration purposes
# Launch from Earth to Mars
launch_time1 = 1 * 365 * 24 * 3600  # Launch after 1 year
landing_time1 = 1.5 * 365 * 24 * 3600  # Land on Mars after 1.5 years
launch_time2 = 3 * 365 * 24 * 3600  # Launch from Mars after 3 years
landing_time2 = 3.5 * 365 * 24 * 3600  # Land on Earth after 3.5 years

x_spacecraft = np.zeros_like(t)
y_spacecraft = np.zeros_like(t)
z_spacecraft = np.zeros_like(t)

# Launch from Earth to Mars
idx_launch1 = np.argmin(np.abs(t - launch_time1))
idx_landing1 = np.argmin(np.abs(t - landing_time1))
x_spacecraft[idx_launch1:idx_landing1] = np.linspace(x_earth[idx_launch1], x_mars[idx_landing1], idx_landing1 - idx_launch1)
y_spacecraft[idx_launch1:idx_landing1] = np.linspace(y_earth[idx_launch1], y_mars[idx_landing1], idx_landing1 - idx_launch1)
z_spacecraft[idx_launch1:idx_landing1] = np.linspace(z_earth[idx_launch1], z_mars[idx_landing1], idx_landing1 - idx_launch1)

# Stay on Mars
idx_stay_mars_start = idx_landing1
idx_stay_mars_end = np.argmin(np.abs(t - launch_time2))
x_spacecraft[idx_stay_mars_start:idx_stay_mars_end] = x_mars[idx_stay_mars_start:idx_stay_mars_end]
y_spacecraft[idx_stay_mars_start:idx_stay_mars_end] = y_mars[idx_stay_mars_start:idx_stay_mars_end]
z_spacecraft[idx_stay_mars_start:idx_stay_mars_end] = z_mars[idx_stay_mars_start:idx_stay_mars_end]

# Launch from Mars to Earth
idx_launch2 = idx_stay_mars_end
idx_landing2 = np.argmin(np.abs(t - landing_time2))
x_spacecraft[idx_launch2:idx_landing2] = np.linspace(x_mars[idx_launch2], x_earth[idx_landing2], idx_landing2 - idx_launch2)
y_spacecraft[idx_launch2:idx_landing2] = np.linspace(y_mars[idx_launch2], y_earth[idx_landing2], idx_landing2 - idx_launch2)
z_spacecraft[idx_launch2:idx_landing2] = np.linspace(z_mars[idx_launch2], z_earth[idx_landing2], idx_landing2 - idx_launch2)


# Create 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.1 * AU, 0.1 * AU])
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')

# Initialize lines
line_earth, = ax.plot([], [], [], 'b', label='Earth')
line_mars, = ax.plot([], [], [], 'r', label='Mars')
line_spacecraft, = ax.plot([], [], [], 'g', label='Spacecraft')

# Legend
ax.legend()


# Animation function
def update(frame):
    line_earth.set_data(x_earth[:frame], y_earth[:frame])
    line_earth.set_3d_properties(z_earth[:frame])

    line_mars.set_data(x_mars[:frame], y_mars[:frame])
    line_mars.set_3d_properties(z_mars[:frame])

    line_spacecraft.set_data(x_spacecraft[:frame], y_spacecraft[:frame])
    line_spacecraft.set_3d_properties(z_spacecraft[:frame])

    return line_earth, line_mars, line_spacecraft


# Create animation
ani = animation.FuncAnimation(fig, update, frames=len(t), interval=20, blit=True)

# Show the animation
plt.show()
