import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Earth's and Mars's average orbital radii in arbitrary units
earth_radius = 1.0
mars_radius = 1.52

# Number of frames for each leg of the journey
frames_per_leg = 100

# Total frames: Earth to Mars, Mars orbit, Mars to Earth (next window)
total_frames = frames_per_leg * 3

# Compute angles for each phase
angles = np.linspace(0, 2*np.pi, total_frames)

# Define positions for Earth and Mars at different points
def get_earth_position(t_index):
    # Earth is relatively stationary for simplicity
    return np.array([0, 0, 0])

def get_mars_position(t_index, phase):
    # Mars moves in a circular orbit
    angle = angles[t_index]
    if phase == 'to_mars' or phase == 'return':
        radius = mars_radius
        # Mars's orbit angle over time
        orbit_angle = angle * 0.5  # Arbitrary angular speed
        x = radius * np.cos(orbit_angle)
        y = radius * np.sin(orbit_angle)
        z = 0
        return np.array([x, y, z])
    elif phase == 'orbiting':
        # For simplicity, keep Mars at a fixed point in orbit
        orbit_angle = angles[frames_per_leg]
        x = mars_radius * np.cos(orbit_angle)
        y = mars_radius * np.sin(orbit_angle)
        z = 0
        return np.array([x, y, z])

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot Earth and Mars
earth_plot, = ax.plot([], [], [], 'bo', label='Earth')
mars_plot, = ax.plot([], [], [], 'ro', label='Mars')
launch_path, = ax.plot([], [], [], 'g--', linewidth=1)

# Set plot limits
max_distance = mars_radius + 0.5
ax.set_xlim(-max_distance, max_distance)
ax.set_ylim(-max_distance, max_distance)
ax.set_zlim(-max_distance/2, max_distance/2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

def init():
    earth_plot.set_data([], [])
    earth_plot.set_3d_properties([])
    mars_plot.set_data([], [])
    mars_plot.set_3d_properties([])
    launch_path.set_data([], [])
    launch_path.set_3d_properties([])
    return earth_plot, mars_plot, launch_path

def animate(i):
    if i < frames_per_leg:
        # Launch from Earth to Mars
        phase = 'to_mars'
        t_idx = i
        # Path from Earth to Mars
        start = get_earth_position(0)
        end = get_mars_position(t_idx, phase)
        t = np.linspace(0, 1, frames_per_leg)
        pos = start.reshape(3,1) * (1 - t) + end.reshape(3,1) * t
        # Update launch path
        launch_path.set_data(pos[0], pos[1])
        launch_path.set_3d_properties(pos[2])
        # Positions
        earth_plot.set_data([[start[0]]], [[start[1]]]])
        earth_plot.set_3d_properties([start[2]])
        mars_plot.set_data([[end[0]]], [[end[1]]]])
        mars_plot.set_3d_properties([end[2]])
    elif i < 2*frames_per_leg:
        # Mars orbiting
        phase = 'orbiting'
        t_idx = i - frames_per_leg
        # Keep Earth at origin
        start = get_earth_position(0)
        # Mars at orbit
        end = get_mars_position(0, phase)
        # Plot Mars and path
        # Position of Mars in orbit
        orbit_angle = angles[frames_per_leg + t_idx] * 0.5
        x = mars_radius * np.cos(orbit_angle)
        y = mars_radius * np.sin(orbit_angle)
        z = 0
        mars_plot.set_data([[x]], [[y]])
        mars_plot.set_3d_properties([z])
        # No launch path during orbit
        launch_path.set_data([], [])
        launch_path.set_3d_properties([])
        # Earth remains at origin
        earth_plot.set_data([[0]], [[0]])
        earth_plot.set_3d_properties([0])
    else:
        # Return trip from Mars to Earth
        phase = 'return'
        t_idx = i - 2*frames_per_leg
        start = get_mars_position(0, 'orbiting')
        end = get_earth_position(t_idx)
        t = np.linspace(0, 1, frames_per_leg)
        pos = start.reshape(3,1) * (1 - t) + end.reshape(3,1) * t
        # Update launch path
        launch_path.set_data(pos[0], pos[1])
        launch_path.set_3d_properties(pos[2])
        # Positions
        mars_plot.set_data([[start[0]]], [[start[1]]]])
        mars_plot.set_3d_properties([start[2]])
        earth_plot.set_data([[end[0]]], [[end[1]]]])
        earth_plot.set_3d_properties([end[2]])
    return earth_plot, mars_plot, launch_path

# Create animation
ani = FuncAnimation(fig, animate, init_func=init,
                    frames=total_frames, interval=50, blit=True, repeat=True)

plt.show()
