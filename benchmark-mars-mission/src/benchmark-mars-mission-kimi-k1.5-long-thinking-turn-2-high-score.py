import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1  # Astronomical Unit
T1 = 258  # Days for Earth-Mars transfer
T_wait = 780  # Days waiting on Mars
T2 = 258  # Days for Mars-Earth transfer
total_days = T1 + T_wait + T2

# Precompute key positions
earth_departure = np.array([1, 0, 0])  # Earth position at departure
mars_arrival = None  # Will be calculated in animation
mars_departure = None  # Will be calculated in animation
earth_arrival = None  # Will be calculated in animation

# Functions to get planet positions
def get_earth_position(t):
    angle = 2 * np.pi * (t % 365) / 365
    return np.array([np.cos(angle), np.sin(angle), 0])

def get_mars_position(t):
    angle = 2 * np.pi * (t % 687) / 687
    return np.array([1.524 * np.cos(angle), 1.524 * np.sin(angle), 0])

# Setup plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel('X (AU)')
ax.set_ylabel('Y (AU)')
ax.set_zlabel('Z (AU)')
ax.set_title('Earth-Mars-Earth Round Trip Mission')

# Plot orbits
theta = np.linspace(0, 2*np.pi, 100)
ax.plot(np.cos(theta), np.sin(theta), zs=0, zdir='z', label='Earth Orbit', color='blue', alpha=0.3)
ax.plot(1.524*np.cos(theta), 1.524*np.sin(theta), zs=0, zdir='z', label='Mars Orbit', color='red', alpha=0.3)

# Initialize objects
earth_obj, = ax.plot([], [], [], 'bo', markersize=10, label='Earth')
mars_obj, = ax.plot([], [], [], 'ro', markersize=10, label='Mars')
spacecraft_obj, = ax.plot([], [], [], 'go', markersize=5, label='Spacecraft')
trajectory, = ax.plot([], [], [], 'g-', alpha=0.7, label='Trajectory')

ax.legend(loc='upper right')

# Animation update function
def update(frame):
    global mars_arrival, mars_departure, earth_arrival
    current_time = frame * (total_days / 300)  # 300 frames total
    
    # Update planet positions
    earth_pos = get_earth_position(current_time)
    mars_pos = get_mars_position(current_time)
    
    earth_obj.set_data([earth_pos[0]], [earth_pos[1]])
    earth_obj.set_3d_properties([earth_pos[2]])
    
    mars_obj.set_data([mars_pos[0]], [mars_pos[1]])
    mars_obj.set_3d_properties([mars_pos[2]])
    
    # Calculate key positions on first pass
    if frame == 0:
        mars_arrival = get_mars_position(T1)
        mars_departure = get_mars_position(T1 + T_wait)
        earth_arrival = get_earth_position(total_days)
    
    # Update spacecraft position
    if current_time <= T1:
        # Earth to Mars transfer
        t = current_time / T1
        sc_pos = earth_departure + t * (mars_arrival - earth_departure)
    elif current_time <= T1 + T_wait:
        # On Mars surface
        sc_pos = mars_pos
    else:
        # Mars to Earth transfer
        t = (current_time - (T1 + T_wait)) / T2
        sc_pos = mars_departure + t * (earth_arrival - mars_departure)
    
    spacecraft_obj.set_data([sc_pos[0]], [sc_pos[1]])
    spacecraft_obj.set_3d_properties([sc_pos[2]])
    
    # Update trajectory
    trajectory.set_data([sc_pos[0]], [sc_pos[1]])
    trajectory.set_3d_properties([sc_pos[2]])
    
    return earth_obj, mars_obj, spacecraft_obj, trajectory

# Animation
ani = FuncAnimation(fig, update, frames=300, interval=50, blit=True)

plt.show()