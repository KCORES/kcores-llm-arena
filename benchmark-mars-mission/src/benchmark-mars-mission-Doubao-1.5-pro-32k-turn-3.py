import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Orbital parameters (approximate)
# Semi - major axes of Earth and Mars orbits in astronomical units (AU)
a_earth = 1.0
a_mars = 1.524

# Orbital periods in years
T_earth = 1.0
T_mars = 1.881

# Generate positions of Earth and Mars over time
t = np.linspace(0, 3 * T_mars, 300)
theta_earth = 2 * np.pi * t / T_earth
theta_mars = 2 * np.pi * t / T_mars

x_earth = a_earth * np.cos(theta_earth)
y_earth = a_earth * np.sin(theta_earth)
z_earth = np.zeros_like(x_earth)

x_mars = a_mars * np.cos(theta_mars)
y_mars = a_mars * np.sin(theta_mars)
z_mars = np.zeros_like(x_mars)

# Approximate launch and return times
launch_time = 0.5
return_time = 2.0

# Interpolation for the spacecraft trajectory
def spacecraft_trajectory(t):
    if t < launch_time:
        x = a_earth * np.cos(2 * np.pi * t / T_earth)
        y = a_earth * np.sin(2 * np.pi * t / T_earth)
        z = 0
    elif t < return_time:
        # Simple linear interpolation from Earth to Mars
        t_fraction = (t - launch_time) / (return_time - launch_time)
        x = (1 - t_fraction) * a_earth * np.cos(2 * np.pi * launch_time / T_earth) + t_fraction * a_mars * np.cos(2 * np.pi * return_time / T_mars)
        y = (1 - t_fraction) * a_earth * np.sin(2 * np.pi * launch_time / T_earth) + t_fraction * a_mars * np.sin(2 * np.pi * return_time / T_mars)
        z = 0
    else:
        # Simple linear interpolation from Mars to Earth
        t_fraction = (t - return_time) / (t[-1] - return_time)
        x = (1 - t_fraction) * a_mars * np.cos(2 * np.pi * return_time / T_mars) + t_fraction * a_earth * np.cos(2 * np.pi * t[-1] / T_earth)
        y = (1 - t_fraction) * a_mars * np.sin(2 * np.pi * return_time / T_mars) + t_fraction * a_earth * np.sin(2 * np.pi * t[-1] / T_earth)
        z = 0
    return x, y, z

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the orbits
ax.plot(x_earth, y_earth, z_earth, label='Earth Orbit', color='blue')
ax.plot(x_mars, y_mars, z_mars, label='Mars Orbit', color='red')

# Initialize the points for Earth, Mars, and the spacecraft
point_earth, = ax.plot([], [], [], 'bo', markersize=10)
point_mars, = ax.plot([], [], [], 'ro', markersize=10)
point_spacecraft, = ax.plot([], [], [], 'go', markersize=5)

# Set plot limits and labels
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-0.1, 0.1])
ax.set_xlabel('X (AU)')
ax.set_ylabel('Y (AU)')
ax.set_zlabel('Z (AU)')
ax.legend()

# Animation update function
def update(frame):
    # Update Earth position
    x_e = a_earth * np.cos(2 * np.pi * t[frame] / T_earth)
    y_e = a_earth * np.sin(2 * np.pi * t[frame] / T_earth)
    z_e = 0
    point_earth.set_data([x_e], [y_e])
    point_earth.set_3d_properties([z_e])

    # Update Mars position
    x_m = a_mars * np.cos(2 * np.pi * t[frame] / T_mars)
    y_m = a_mars * np.sin(2 * np.pi * t[frame] / T_mars)
    z_m = 0
    point_mars.set_data([x_m], [y_m])
    point_mars.set_3d_properties([z_m])

    # Update spacecraft position
    x_s, y_s, z_s = spacecraft_trajectory(t[frame])
    point_spacecraft.set_data([x_s], [y_s])
    point_spacecraft.set_3d_properties([z_s])

    return point_earth, point_mars, point_spacecraft

# Create the animation
ani = FuncAnimation(fig, update, frames=len(t), interval=20, blit=True)

# Save the animation as a video
ani.save('spacecraft_launch.mp4', writer='ffmpeg', fps=30)

# Show the animation
plt.show()
