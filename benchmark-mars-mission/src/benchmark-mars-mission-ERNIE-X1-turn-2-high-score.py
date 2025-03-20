import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
G = 6.67430e-11  # Gravitational constant (m³/kg/s²)
AU = 1.496e11    # Astronomical Unit (meters)
days_per_year = 365

# Simplified orbital parameters (Earth: 1 AU, Mars: 1.5 AU)
T_earth = 365 * 24 * 3600  # Earth's orbital period (seconds)
T_mars = 687 * 24 * 3600   # Mars' orbital period (seconds)
transfer_time = 250 * 24 * 3600  # Transfer duration (seconds)
synodic_period = 780 * 24 * 3600  # Time between launch windows

# Convert to days for easier handling
T_earth /= 24 * 3600
T_mars /= 24 * 3600
transfer_time /= 24 * 3600
synodic_period /= 24 * 3600

# Precompute key times
t_launch_earth = 0
t_arrive_mars = t_launch_earth + transfer_time
t_launch_mars = t_arrive_mars + synodic_period
t_arrive_earth = t_launch_mars + transfer_time

# Initialize storage for spacecraft trail
spacecraft_trail = []
max_trail_length = 200

def get_position(planet, t):
    """Calculate orbital position for a planet"""
    theta = 2 * np.pi * t / planet['period']
    radius = planet['radius']
    return np.array([
        radius * np.cos(theta),
        radius * np.sin(theta),
        0  # All in the same plane
    ])

# Planet definitions
planets = {
    'sun': {'radius': 0, 'period': 0, 'color': 'gold', 'size': 200},
    'earth': {'radius': 1, 'period': T_earth, 'color': 'blue', 'size': 80},
    'mars': {'radius': 1.5, 'period': T_mars, 'color': 'red', 'size': 60}
}

# Precompute transfer endpoints
earth_start = get_position(planets['earth'], t_launch_earth)
mars_arrival = get_position(planets['mars'], t_arrive_mars)
mars_start = get_position(planets['mars'], t_launch_mars)
earth_arrival = get_position(planets['earth'], t_arrive_earth)

def animate(t):
    ax.clear()
    
    # Draw Sun
    ax.scatter([0], [0], [0], c=planets['sun']['color'], s=planets['sun']['size'])
    
    # Draw planets
    for planet in ['earth', 'mars']:
        pos = get_position(planets[planet], t)
        ax.scatter(pos[0], pos[1], pos[2], 
                  c=planets[planet]['color'], 
                  s=planets[planet]['size'])
    
    # Calculate spacecraft position
    if t < t_arrive_mars:
        # Traveling to Mars
        frac = t / transfer_time
        pos = earth_start + (mars_arrival - earth_start) * frac
    elif t < t_launch_mars:
        # On Mars surface
        pos = get_position(planets['mars'], t)
    else:
        # Returning to Earth
        frac = (t - t_launch_mars) / transfer_time
        pos = mars_start + (earth_arrival - mars_start) * frac
    
    # Draw spacecraft and trail
    spacecraft_trail.append(pos)
    if len(spacecraft_trail) > max_trail_length:
        spacecraft_trail.pop(0)
    
    trail = np.array(spacecraft_trail)
    ax.scatter(pos[0], pos[1], pos[2], c='green', s=40)
    ax.plot(trail[:,0], trail[:,1], trail[:,2], c='gray', alpha=0.3)
    
    # Set axes limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-0.5, 0.5])
    
    # Add labels
    ax.set_xlabel('X (AU)')
    ax.set_ylabel('Y (AU)')
    ax.set_zlabel('Z (AU)')
    ax.set_title(f'Day: {int(t)}')
    
    return []

# Create figure and animation
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ani = FuncAnimation(fig, animate, 
                   frames=np.linspace(0, t_arrive_earth, int(t_arrive_earth)),
                   interval=50, repeat=False)

plt.show()