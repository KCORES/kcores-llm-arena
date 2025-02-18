import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# Constants (simplified for demonstration)
G = 6.6743e-11  # Gravitational constant (m^3 kg^-1 s^-2)
M_earth = 5.972e24  # Earth mass (kg)
M_mars = 6.39e23  # Mars mass (kg)
R_earth = 6371e3  # Earth radius (m)
R_mars = 3389.5e3  # Mars radius (m)
AU = 149.6e9  # Astronomical Unit (m) - Earth-Sun distance

# Launch parameters
v_esc_earth = np.sqrt(2 * G * M_earth / R_earth)  # Earth escape velocity
# Simplified delta-v for Hohmann transfer - very rough estimate
delta_v_launch = v_esc_earth * 1.2  # Add a little extra for maneuver
delta_v_mars_capture = 3000  # m/s  Roughly estimates capture burn, not exact
delta_v_earth_capture = 3000  # m/s

# Time parameters
days_to_mars = (
    200  # Roughly estimate, not accurate based on Hohmann transfer calculations
)
days_on_mars = 400  # Time spent on Mars.  Could be improved.
days_return = 280  # Roughly estimate time for the return Hohmann transfer.
dt = (
    86400 / 10
)  # Time step (seconds), make shorter for better animation, use real orbit sim for accuracy.
total_time = (days_to_mars + days_on_mars + days_return) * 86400  # total flight time
num_frames = int(total_time / dt)

# Initial conditions (simplified)
# Earth orbit: circular for simplicity, use real orbital data for accuracy
earth_orbit_radius = AU
earth_orbital_speed = np.sqrt(
    G * 1e30 / earth_orbit_radius
)  # Using a star mass approximation

# Mars orbit (circular approximation as well)
mars_orbit_radius = 1.524 * AU
mars_orbital_speed = np.sqrt(
    G * 1e30 / mars_orbit_radius
)  # Using a star mass approximation


# Initialize positions and velocities - starting near Earth

pos_earth = np.array([earth_orbit_radius, 0, 0])
vel_earth = np.array([0, earth_orbital_speed, 0])

pos_mars = np.array(
    [
        mars_orbit_radius * np.cos(2 * np.pi * (days_to_mars / 365)),
        mars_orbit_radius * np.sin(2 * np.pi * (days_to_mars / 365)),
        0,
    ]
)
vel_mars = np.array(
    [
        -mars_orbital_speed * np.sin(2 * np.pi * (days_to_mars / 365)),
        mars_orbital_speed * np.cos(2 * np.pi * (days_to_mars / 365)),
        0,
    ]
)

pos_ship = np.array([R_earth + 100000, 0, 0])  # Start at orbit, 100km above Earth
vel_ship = np.array(
    [0, earth_orbital_speed + delta_v_launch / 3, 0]
)  # Initially match Earth's velocity, add launch delta-v component
# 3/2.5 just to get the initial starting direction.  These are not accurate, but good to start
# for a demo
# Initialize trajectory arrays
ship_trajectory = np.zeros((num_frames, 3))
earth_trajectory = np.zeros((num_frames, 3))
mars_trajectory = np.zeros((num_frames, 3))

# Animation setup
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot limits (adjust these based on your orbit parameters)
ax.set_xlim(
    [-1.8 * mars_orbit_radius, 1.8 * mars_orbit_radius]
)  # Scale to the Mars orbit
ax.set_ylim([-1.8 * mars_orbit_radius, 1.8 * mars_orbit_radius])
ax.set_zlim([-AU / 10, AU / 10])  # Reduced to see everything.

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Mars Mission Animation")

# Objects
(earth,) = ax.plot([], [], [], "g.", markersize=10, label="Earth")
(mars,) = ax.plot([], [], [], "r.", markersize=8, label="Mars")
(ship,) = ax.plot(
    [], [], [], "b-", linewidth=1, label="Spacecraft"
)  # reduced the linewidth for clarity
(ship_path,) = ax.plot(
    [], [], [], "b-", linewidth=0.5, alpha=0.3
)  # Reduced for clarity.
(sun,) = ax.plot([0], [0], [0], "y*", markersize=15, label="Sun")


# Trajectory calculations (simplified, need a proper orbital mechanics implementation for accuracy!)
def update(frame):
    global pos_ship, vel_ship, pos_earth, vel_earth, pos_mars, vel_mars

    # Earth orbit update
    # For better sim, would need the sun and Mars influence, and keep track of the real
    # location of Earth, not simplified.
    earth_acc = -G * 1e30 * pos_earth / np.linalg.norm(pos_earth) ** 3
    vel_earth = vel_earth + earth_acc * dt
    pos_earth = pos_earth + vel_earth * dt

    # Mars orbit update
    mars_acc = -G * 1e30 * pos_mars / np.linalg.norm(pos_mars) ** 3
    vel_mars = vel_mars + mars_acc * dt
    pos_mars = pos_mars + vel_mars * dt

    # Spacecraft trajectory (Simplified, very rudimentary.  Real simulation would use full n-body gravity)
    earth_ship_vector = pos_ship - pos_earth
    mars_ship_vector = pos_ship - pos_mars

    # Earth's Gravity
    earth_acc_ship = (
        -G * M_earth * earth_ship_vector / np.linalg.norm(earth_ship_vector) ** 3
    )
    # Mars Gravity
    mars_acc_ship = (
        -G * M_mars * mars_ship_vector / np.linalg.norm(mars_ship_vector) ** 3
    )

    ship_acc = earth_acc_ship + mars_acc_ship
    vel_ship = vel_ship + ship_acc * dt
    pos_ship = pos_ship + vel_ship * dt

    # Capture Burns, and return burns would happen at the appropriate times
    # based on delta-v maneuvers, and relative locations of the planets
    # Simplification:
    if frame > days_to_mars * 86400 / dt:
        # This would be the "capture burn"
        vel_ship = (
            vel_ship - vel_ship / np.linalg.norm(vel_ship) * delta_v_mars_capture
        )  # Change direction
    if frame > (days_to_mars + days_on_mars) * 86400 / dt:
        # Return
        earth_ship_vector = pos_ship - pos_earth  # Use for direction
        vel_ship = (
            vel_ship
            - earth_ship_vector
            / np.linalg.norm(earth_ship_vector)
            * delta_v_mars_capture
        )  # Change direction

    ship_trajectory[frame, :] = pos_ship
    earth_trajectory[frame, :] = pos_earth
    mars_trajectory[frame, :] = pos_mars

    # Update the plots
    earth.set_data([pos_earth[0], pos_earth[1]])
    earth.set_3d_properties([pos_earth[2]])

    mars.set_data([pos_mars[0], pos_mars[1]])
    mars.set_3d_properties([pos_mars[2]])

    ship.set_data([pos_ship[0], pos_ship[1]])
    ship.set_3d_properties([pos_ship[2]])

    ship_path.set_data(ship_trajectory[:frame, 0], ship_trajectory[:frame, 1])
    ship_path.set_3d_properties(ship_trajectory[:frame, 2])
    ax.legend()
    return (
        earth,
        mars,
        ship,
        ship_path,
    )


# Animation
ani = animation.FuncAnimation(
    fig, update, frames=num_frames, blit=False, interval=20, repeat=False
)

# Save animation (optional, requires ffmpeg or other codec)
# ani.save("mars_mission.mp4", writer='ffmpeg', fps=30)

plt.show()
