import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e8  # km
DAY = 24*3600  # seconds
YEAR = 365.25*DAY

# Orbital parameters
earth_orbit = 1.0 * AU
mars_orbit = 1.524 * AU
earth_period = 1 * YEAR
mars_period = 1.88 * YEAR

# Hohmann transfer parameters
transfer_a = (earth_orbit + mars_orbit)/2
transfer_period = 2*np.pi*np.sqrt(transfer_a**3/(1.327e11))  # Seconds
transfer_time = transfer_period/2  # Half-orbit time

# Time arrays
t_outbound = np.linspace(0, transfer_time, 200)
t_return = np.linspace(transfer_time, 2*transfer_time, 200)

# Planet positions (simplified circular orbits)
def planet_position(t, orbit, period):
    angle = 2*np.pi * (t % period)/period
    return orbit*np.cos(angle), orbit*np.sin(angle), 0

# Spacecraft trajectory (Hohmann transfer)
def hohmann_position(t, start_time, direction=1):
    if direction == 1:  # Earth -> Mars
        a = transfer_a
        theta = np.pi * (t - start_time)/transfer_time
    else:  # Mars -> Earth
        a = transfer_a
        theta = np.pi * (1 - (t - start_time)/transfer_time)
    
    r = a*(1 - np.cos(theta)) + earth_orbit
    x = r*np.cos(theta)
    y = r*np.sin(theta)
    return x, y, 0

# Initialize plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2*AU, 2*AU)
ax.set_ylim(-2*AU, 2*AU)
ax.set_zlim(-0.5*AU, 0.5*AU)
ax.set_xlabel('X (km)')
ax.set_ylabel('Y (km)')
ax.set_zlabel('Z (km)')
ax.set_title('Earth-Mars Round Trip Mission')

# Plot elements
earth, = ax.plot([], [], [], 'bo', label='Earth')
mars, = ax.plot([], [], [], 'ro', label='Mars')
spacecraft, = ax.plot([], [], [], 'go', label='Spacecraft')
trajectory, = ax.plot([], [], [], 'g-', alpha=0.5)
time_text = ax.text(0.05, 0.95, 0.95, '', transform=ax.transAxes)

# Initialize function
def init():
    earth.set_data([], [])
    mars.set_data([], [])
    spacecraft.set_data([], [])
    trajectory.set_data([], [])
    time_text.set_text('')
    return earth, mars, spacecraft, trajectory, time_text

# Update function
def update(frame):
    t = frame * transfer_time / 199  # Current time
    
    # Planet positions
    ex, ey, ez = planet_position(t, earth_orbit, earth_period)
    mx, my, mz = planet_position(t, mars_orbit, mars_period)
    
    # Spacecraft position
    if t < transfer_time:
        sx, sy, sz = hohmann_position(t, 0, 1)
    else:
        sx, sy, sz = hohmann_position(t, transfer_time, -1)
    
    # Update plots
    earth.set_data([ex/AU], [ey/AU])
    earth.set_3d_properties(ez/AU)
    
    mars.set_data([mx/AU], [my/AU])
    mars.set_3d_properties(mz/AU)
    
    spacecraft.set_data([sx/AU], [sy/AU])
    spacecraft.set_3d_properties(sz/AU)
    
    # Update trajectory
    if frame == 0:
        trajectory.set_data([], [])
    else:
        traj_x = np.append(trajectory.get_xdata(), sx/AU)
        traj_y = np.append(trajectory.get_ydata(), sy/AU)
        trajectory.set_data(traj_x, traj_y)
        trajectory.set_3d_properties(np.append(trajectory.get_3d_properties(), sz/AU))
    
    time_text.set_text(f'Time: {t/(365*24*3600):.1f} years')
    
    return earth, mars, spacecraft, trajectory, time_text

# Create animation
ani = FuncAnimation(fig, update, frames=400,
                    init_func=init, blit=True, interval=50)

plt.legend(loc='upper right')
plt.show()