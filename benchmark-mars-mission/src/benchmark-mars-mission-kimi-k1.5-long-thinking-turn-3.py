import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Orbital parameters (simplified)
earth_orbit = 1.0  # AU
mars_orbit = 1.524  # AU
synodic_period = 780  # days between launch windows

# Time settings
total_days = 2 * synodic_period  # ~2.15 years
time = np.linspace(0, total_days, 1000)
delta_t = 0.5  # days per frame

# Calculate planet positions
def planet_position(orbit_radius, orbital_period, time):
    angle = 2 * np.pi * (time % orbital_period) / orbital_period
    return orbit_radius * np.cos(angle), orbit_radius * np.sin(angle)

earth_x, earth_y = planet_position(earth_orbit, 365.25, time)
mars_x, mars_y = planet_position(mars_orbit, 687, time)

# Trajectory calculation (simplified Hohmann transfer)
def hohmann_transfer(t0, dt, r1, r2):
    a = (r1 + r2) / 2
    transfer_period = np.sqrt(a**3)  # In years (simplified)
    transfer_time = transfer_period / 2  # Half orbit for transfer
    angle = np.linspace(0, np.pi, int(dt/delta_t))
    x = r1 * np.cos(angle) + (r2 - r1) * (angle/np.pi)
    y = np.sqrt(r1 * r2) * np.sin(angle)
    return x, y

# Create trajectory segments
departure_time = 0
transfer1_x, transfer1_y = hohmann_transfer(departure_time, synodic_period/2, earth_orbit, mars_orbit)
surface_time = 30  # Days on Mars
return_x, return_y = hohmann_transfer(departure_time + synodic_period/2 + surface_time, 
                                     synodic_period/2, mars_orbit, earth_orbit)

# Combine trajectories
trajectory_x = np.concatenate([transfer1_x, np.full(surface_time/delta_t, mars_x[-1]), return_x])
trajectory_y = np.concatenate([transfer1_y, np.full(surface_time/delta_t, mars_y[-1]), return_y])
trajectory_z = np.zeros_like(trajectory_x)

# Setup plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X (AU)')
ax.set_ylabel('Y (AU)')
ax.set_zlabel('Z (AU)')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)

# Plot orbits
ax.plot(earth_x, earth_y, 0, color='blue', alpha=0.3, label='Earth Orbit')
ax.plot(mars_x, mars_y, 0, color='red', alpha=0.3, label='Mars Orbit')

# Initialize elements
earth_dot, = ax.plot([], [], [], 'o', color='blue', markersize=10, label='Earth')
mars_dot, = ax.plot([], [], [], 'o', color='red', markersize=8, label='Mars')
spacecraft, = ax.plot([], [], [], '^-', color='green', markersize=8, label='Spacecraft')
trajectory, = ax.plot([], [], [], color='gray', alpha=0.7)

def init():
    earth_dot.set_data([], [])
    mars_dot.set_data([], [])
    spacecraft.set_data([], [])
    trajectory.set_data([], [])
    return earth_dot, mars_dot, spacecraft, trajectory

def animate(i):
    t = i * delta_t
    idx = np.searchsorted(time, t)
    
    # Update planet positions
    earth_dot.set_data([earth_x[idx]], [earth_y[idx]])
    earth_dot.set_3d_properties(0)
    
    mars_dot.set_data([mars_x[idx]], [mars_y[idx]])
    mars_dot.set_3d_properties(0)
    
    # Update spacecraft position
    if t < len(trajectory_x)*delta_t:
        traj_idx = int(t/delta_t)
        spacecraft.set_data(trajectory_x[traj_idx], trajectory_y[traj_idx])
        spacecraft.set_3d_properties(trajectory_z[traj_idx])
        
        # Show trajectory trail
        trail = max(0, traj_idx - 20)
        trajectory.set_data(trajectory_x[trail:traj_idx], trajectory_y[trail:traj_idx])
        trajectory.set_3d_properties(trajectory_z[trail:traj_idx])
    
    return earth_dot, mars_dot, spacecraft, trajectory

ani = FuncAnimation(fig, animate, frames=int(total_days/delta_t), 
                    init_func=init, blit=True, interval=20)

plt.legend(loc='upper right')
plt.show()