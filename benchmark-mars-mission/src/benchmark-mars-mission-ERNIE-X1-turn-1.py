import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants
R_SUN = 0.0  # Sun at origin
R_EARTH = 1.0  # Earth's orbital radius (AU)
R_MARS = 1.5  # Mars's orbital radius (AU)
T_EARTH = 1.0  # Earth's orbital period (years)
T_MARS = 1.88  # Mars's orbital period (years)
TOTAL_FRAMES = 1000  # Total animation frames
STEP_SIZE = 0.05  # Spacecraft movement step (AU/frame)
STAY_DURATION = 50  # Frames to stay at destination

# Initialization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Initial positions
earth_pos = np.array([R_EARTH, 0, 0])
mars_pos = np.array([R_MARS, 0, 0])
spacecraft_pos = earth_pos.copy()

is_going_to_mars = True
frames_since_launch = 0
frames_at_destination = 0

def update(frame):
    global earth_pos, mars_pos, spacecraft_pos
    global is_going_to_mars, frames_since_launch, frames_at_destination
    
    # Update planetary positions
    theta_earth = 2 * np.pi * frame / T_EARTH / TOTAL_FRAMES
    earth_pos = np.array([
        R_EARTH * np.cos(theta_earth),
        R_EARTH * np.sin(theta_earth),
        0
    ])
    
    theta_mars = 2 * np.pi * frame / T_MARS / TOTAL_FRAMES
    mars_pos = np.array([
        R_MARS * np.cos(theta_mars),
        R_MARS * np.sin(theta_mars),
        0
    ])
    
    # Update spacecraft position
    if is_going_to_mars:
        direction = mars_pos - spacecraft_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0.01:
            spacecraft_pos += STEP_SIZE * direction / distance
        else:
            frames_at_destination += 1
            if frames_at_destination > STAY_DURATION:
                is_going_to_mars = False
                frames_since_launch = 0
                frames_at_destination = 0
    else:
        direction = earth_pos - spacecraft_pos
        distance = np.linalg.norm(direction)
        
        if distance > 0.01:
            spacecraft_pos += STEP_SIZE * direction / distance
        else:
            spacecraft_pos = earth_pos.copy()
            is_going_to_mars = True
            frames_since_launch = 0
            frames_at_destination = 0

    # Clear frame
    ax.clear()
    
    # Set axis limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-1, 1])
    
    # Plot objects
    ax.scatter([R_SUN], [R_SUN], [R_SUN], c='yellow', s=200, label='Sun')
    ax.scatter(earth_pos[0], earth_pos[1], earth_pos[2], c='blue', s=100, label='Earth')
    ax.scatter(mars_pos[0], mars_pos[1], mars_pos[2], c='red', s=100, label='Mars')
    ax.scatter(spacecraft_pos[0], spacecraft_pos[1], spacecraft_pos[2], 
               c='green', s=50, label='Spacecraft')
    
    # Add labels
    ax.set_xlabel('X (AU)')
    ax.set_ylabel('Y (AU)')
    ax.set_zlabel('Z (AU)')
    ax.legend()
    
    return []

# Create animation
ani = animation.FuncAnimation(fig, update, frames=TOTAL_FRAMES, interval=50)
plt.show()