import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Orbital parameters (AU and days)
a_earth = 1.0        # Earth's semi-major axis (AU)
a_mars = 1.524       # Mars's semi-major axis (AU)
period_earth = 365   # Earth's orbital period (days)
period_mars = 687    # Mars's orbital period (days)

# Transfer parameters
transfer_time = 250  # Days for each transfer
wait_time = 780      # Days between Mars arrival and return launch
total_time = transfer_time + wait_time + transfer_time

# Initialize figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2.5, 2.5])
ax.set_ylim([-2.5, 2.5])
ax.set_zlim([-1, 1])
ax.set_xlabel('AU')
ax.set_ylabel('AU')
ax.set_zlabel('AU')
ax.set_title('Earth-Mars Mission Animation')

# Create orbit trails
theta = np.linspace(0, 2*np.pi, 100)
earth_orbit = a_earth * np.array([np.cos(theta), np.sin(theta), np.zeros_like(theta)])
mars_orbit = a_mars * np.array([np.cos(theta), np.sin(theta), np.zeros_like(theta)])

# Initialize plot elements
sun = ax.scatter([], [], [], c='gold', s=200, label='Sun')
earth = ax.scatter([], [], [], c='blue', s=100, label='Earth')
mars = ax.scatter([], [], [], c='red', s=100, label='Mars')
sc1 = ax.scatter([], [], [], c='limegreen', s=50, label='Spacecraft 1')
sc2 = ax.scatter([], [], [], c='magenta', s=50, label='Spacecraft 2')
ax.plot(*earth_orbit, color='blue', alpha=0.2)
ax.plot(*mars_orbit, color='red', alpha=0.2)
ax.legend()

# Position calculation functions
def get_position(a, period, t, phase=0):
    omega = 2 * np.pi / period
    theta = phase + omega * t
    return a * np.cos(theta), a * np.sin(theta), 0

# Animation update function
def update(t):
    # Clear previous positions
    sun._offsets3d = ([0, 0, 0],)
    earth._offsets3d = ([], [], [])
    mars._offsets3d = ([], [], [])
    sc1._offsets3d = ([], [], [])
    sc2._offsets3d = ([], [], [])
    
    # Calculate current positions
    x_e, y_e, z_e = get_position(a_earth, period_earth, t, phase=0)
    x_m, y_m, z_m = get_position(a_mars, period_mars, t, phase=np.pi)
    
    # Spacecraft 1 logic
    if 0 <= t <= transfer_time:
        # Transfer from Earth to Mars
        alpha = t / transfer_time
        x_sc1 = x_e_start + alpha * (x_m_arrival - x_e_start)
        y_sc1 = y_e_start + alpha * (y_m_arrival - y_e_start)
        sc1._offsets3d = ([x_sc1], [y_sc1], [0])
    elif t > transfer_time:
        # Landed on Mars
        sc1._offsets3d = ([x_m], [y_m], [0])
    
    # Spacecraft 2 logic
    if t >= transfer_time + wait_time and t <= total_time:
        # Transfer from Mars to Earth
        alpha = (t - (transfer_time + wait_time)) / transfer_time
        x_sc2 = x_m_departure + alpha * (x_e_final - x_m_departure)
        y_sc2 = y_m_departure + alpha * (y_e_final - y_m_departure)
        sc2._offsets3d = ([x_sc2], [y_sc2], [0])
    elif t < transfer_time + wait_time and t >= transfer_time:
        # Waiting on Mars
        sc2._offsets3d = ([x_m], [y_m], [0])
    
    # Update planet positions
    earth._offsets3d = ([x_e], [y_e], [z_e])
    mars._offsets3d = ([x_m], [y_m], [z_m])
    
    # Pre-calculate important positions
    global x_e_start, y_e_start, x_m_arrival, y_m_arrival
    global x_m_departure, y_m_departure, x_e_final, y_e_final
    
    x_e_start, y_e_start, _ = get_position(a_earth, period_earth, 0, phase=0)
    x_m_arrival, y_m_arrival, _ = get_position(a_mars, period_mars, transfer_time, phase=np.pi)
    
    x_m_departure, y_m_departure, _ = get_position(a_mars, period_mars, transfer_time + wait_time, phase=np.pi)
    x_e_final, y_e_final, _ = get_position(a_earth, period_earth, total_time, phase=0)

    return []

# Pre-calculate initial positions
x_e_start, y_e_start, _ = get_position(a_earth, period_earth, 0, phase=0)
x_m_arrival, y_m_arrival, _ = get_position(a_mars, period_mars, transfer_time, phase=np.pi)
x_m_departure, y_m_departure, _ = get_position(a_mars, period_mars, transfer_time + wait_time, phase=np.pi)
x_e_final, y_e_final, _ = get_position(a_earth, period_earth, total_time, phase=0)

# Create animation
ani = FuncAnimation(fig, update, frames=np.linspace(0, total_time, 800), interval=50)

plt.show()