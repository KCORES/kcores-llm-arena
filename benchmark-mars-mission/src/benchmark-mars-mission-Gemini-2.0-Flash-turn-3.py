import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.constants import G  # Gravitational constant

# --- Constants ---
# Using values in SI units (meters, kg, seconds)
# Adjusted for better visual scaling
# Earth
earth_radius = 6.371e6  # meters
earth_mass = 5.972e24  # kg
earth_orbital_radius = 1.496e11  # meters (distance from Sun)
earth_orbital_speed = 29780  # m/s around Sun

# Mars
mars_radius = 3.3895e6  # meters
mars_mass = 6.4171e23  # kg
mars_orbital_radius = 2.279e11  # meters (distance from Sun)
mars_orbital_speed = 24077  # m/s around Sun

# Sun (Simplified as just its position)
sun_position = np.array([0, 0, 0])

# --- Simulation Parameters ---
time_step = 3600  # seconds (1 hour) - Adjust for speed/accuracy
simulation_duration = (
    3 * 365 * 24 * 3600
)  # seconds (3 years) - roughly launch window + journey + wait + journey back
num_frames = 500  # Adjust for animation smoothness; higher = slower animation
frame_interval = simulation_duration / num_frames  # time per frame

# --- Initial Conditions ---
# Adjust these for realistic trajectory (simplified for demo)
# Earth Launch:  Approximate initial velocity components to get to Mars
initial_launch_altitude = earth_radius + 300e3  # meters,  Low Earth Orbit
launch_position_earth = np.array([initial_launch_altitude, 0, 0])
initial_velocity_earth = np.array(
    [0, 11000, 0]
)  # m/s,  Slightly higher than LEO speed, directed towards mars
# Mars Landing:
mars_landing_altitude = mars_radius + 100e3  # meters
# Earth Return:
return_launch_altitude = mars_radius + 300e3
return_launch_position = np.array([return_launch_altitude, 0, 0])  # relative to mars
return_velocity = np.array([0, 8000, 0])  # relative to mars


# --- Planet Positions ---
# Earth and Mars orbit the Sun in the XY plane (simplified)
def planet_position(orbital_radius, orbital_speed, time):
    angle = (time * orbital_speed) / orbital_radius
    x = orbital_radius * np.cos(angle)
    y = orbital_radius * np.sin(angle)
    return np.array([x, y, 0])


def planet_velocity(orbital_radius, orbital_speed, time):
    angle = (time * orbital_speed) / orbital_radius
    x = -orbital_speed * np.sin(angle)
    y = orbital_speed * np.cos(angle)
    return np.array([x, y, 0])


# --- Gravitational Force Function ---
def gravitational_force(position1, mass1, position2, mass2):
    r = position2 - position1
    distance = np.linalg.norm(r)
    force_magnitude = (G * mass1 * mass2) / (distance**2)
    force_direction = r / distance  # Unit vector in the direction of the force
    return force_magnitude * force_direction


# --- Update Function for Spacecraft Position ---
def update_spacecraft_position(
    position, velocity, time_step, earth_position, mars_position
):
    # Calculate gravitational forces
    force_earth = gravitational_force(
        position, 1000, earth_position, earth_mass
    )  # spacecraft mass assumed as 1000kg
    force_mars = gravitational_force(position, 1000, mars_position, mars_mass)
    force_sun = gravitational_force(
        position, 1000, sun_position, 1.989e30
    )  # Sun's mass

    # Net force
    net_force = force_earth + force_mars + force_sun

    # Calculate acceleration
    acceleration = net_force / 1000  # spacecraft mass

    # Update velocity and position using Euler integration (can use more accurate methods)
    velocity += acceleration * time_step
    position += velocity * time_step

    return position, velocity


# --- Simulation ---
spacecraft_positions = []

# Initial Earth position and velocity
earth_pos = planet_position(earth_orbital_radius, earth_orbital_speed, 0)
earth_vel = planet_velocity(earth_orbital_radius, earth_orbital_speed, 0)

# Initial Mars position and velocity
mars_pos = planet_position(mars_orbital_radius, mars_orbital_speed, 0)
mars_vel = planet_velocity(mars_orbital_radius, mars_orbital_speed, 0)

# Launch from Earth
spacecraft_position = launch_position_earth + earth_pos  # add earth's position
spacecraft_velocity = initial_velocity_earth + earth_vel  # add earth's velocity

earth_reached_mars_time = None
mars_launched_time = None

for i in range(num_frames):
    time = i * frame_interval

    # Update planet positions
    earth_pos = planet_position(earth_orbital_radius, earth_orbital_speed, time)
    earth_vel = planet_velocity(earth_orbital_radius, earth_orbital_speed, time)
    mars_pos = planet_position(mars_orbital_radius, mars_orbital_speed, time)
    mars_vel = planet_velocity(mars_orbital_radius, mars_orbital_speed, time)

    # Update spacecraft position
    spacecraft_position, spacecraft_velocity = update_spacecraft_position(
        spacecraft_position, spacecraft_velocity, time_step, earth_pos, mars_pos
    )

    # Check if spacecraft reached Mars (very simplified)
    if (
        earth_reached_mars_time is None
        and np.linalg.norm(spacecraft_position - mars_pos) < mars_landing_altitude + 1e6
    ):  # within 1000km
        earth_reached_mars_time = time
        print(
            "Reached Mars at time:",
            earth_reached_mars_time / (3600 * 24 * 365),
            " years",
        )
        spacecraft_position = (
            mars_pos + return_launch_position
        )  # Place it on Mars' surface
        spacecraft_velocity = mars_vel  # Make it move with Mars

    # Launch from Mars
    if (
        earth_reached_mars_time is not None
        and mars_launched_time is None
        and time > earth_reached_mars_time + 365 * 24 * 3600
    ):  # wait one earth year.
        mars_launched_time = time
        print(
            "Launching from mars at time: ",
            mars_launched_time / (3600 * 24 * 365),
            " years",
        )
        spacecraft_velocity = return_velocity + mars_vel  # Add mars speed
        spacecraft_position = (
            mars_pos + return_launch_position
        )  # launch from same point

    spacecraft_positions.append(spacecraft_position.copy())  # append copy
    # Optionally print progress
    # if i % 10 == 0:
    #    print(f"Frame {i}: Spacecraft Position: {spacecraft_position}")

spacecraft_positions = np.array(spacecraft_positions)

# --- Visualization ---

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot planets (static)
ax.plot(
    [sun_position[0]],
    [sun_position[1]],
    [sun_position[2]],
    "yo",
    markersize=10,
    label="Sun",
)
earth_orbit_x = [
    planet_position(earth_orbital_radius, earth_orbital_speed, t)[0]
    for t in np.linspace(0, simulation_duration, 50)
]
earth_orbit_y = [
    planet_position(earth_orbital_radius, earth_orbital_speed, t)[1]
    for t in np.linspace(0, simulation_duration, 50)
]
earth_orbit_z = [
    planet_position(earth_orbital_radius, earth_orbital_speed, t)[2]
    for t in np.linspace(0, simulation_duration, 50)
]
ax.plot(
    earth_orbit_x, earth_orbit_y, earth_orbit_z, "b-", alpha=0.3, label="Earth Orbit"
)

mars_orbit_x = [
    planet_position(mars_orbital_radius, mars_orbital_speed, t)[0]
    for t in np.linspace(0, simulation_duration, 50)
]
mars_orbit_y = [
    planet_position(mars_orbital_radius, mars_orbital_speed, t)[1]
    for t in np.linspace(0, simulation_duration, 50)
]
mars_orbit_z = [
    planet_position(mars_orbital_radius, mars_orbital_speed, t)[2]
    for t in np.linspace(0, simulation_duration, 50)
]
ax.plot(mars_orbit_x, mars_orbit_y, mars_orbit_z, "r-", alpha=0.3, label="Mars Orbit")

(earth_dot,) = ax.plot(
    [earth_pos[0]], [earth_pos[1]], [earth_pos[2]], "bo", markersize=5, label="Earth"
)
(mars_dot,) = ax.plot(
    [mars_pos[0]], [mars_pos[1]], [mars_pos[2]], "ro", markersize=5, label="Mars"
)


# Spacecraft trail
(spacecraft_trail,) = ax.plot(
    [], [], [], "g-", linewidth=2, label="Spacecraft Trajectory"
)
(spacecraft_dot,) = ax.plot([], [], [], "go", markersize=5, label="Spacecraft")


# --- Animation Function ---
def animate(i):
    time = i * frame_interval
    earth_pos = planet_position(earth_orbital_radius, earth_orbital_speed, time)
    mars_pos = planet_position(mars_orbital_radius, mars_orbital_speed, time)

    earth_dot.set_data([earth_pos[0]], [earth_pos[1]])
    earth_dot.set_3d_properties([earth_pos[2]])

    mars_dot.set_data([mars_pos[0]], [mars_pos[1]])
    mars_dot.set_3d_properties([mars_pos[2]])

    spacecraft_trail.set_data(spacecraft_positions[:i, 0], spacecraft_positions[:i, 1])
    spacecraft_trail.set_3d_properties(spacecraft_positions[:i, 2])

    spacecraft_dot.set_data([spacecraft_positions[i, 0]], [spacecraft_positions[i, 1]])
    spacecraft_dot.set_3d_properties([spacecraft_positions[i, 2]])

    return earth_dot, mars_dot, spacecraft_trail, spacecraft_dot


# --- Plot Settings ---
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Earth-Mars-Earth Mission")
ax.legend(loc="upper left")

# Use a fixed scale to maintain consistent visual proportions.
max_distance = max(earth_orbital_radius, mars_orbital_radius) * 1.2
ax.set_xlim([-max_distance, max_distance])
ax.set_ylim([-max_distance, max_distance])
ax.set_zlim([-max_distance / 10, max_distance / 10])  # Flatten z for visualization

# Turn off axes numbers for aesthetics
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

# --- Run Animation ---
ani = FuncAnimation(fig, animate, frames=num_frames, blit=True, repeat=False)

# Save the animation (optional)
# ani.save('earth_mars_earth_mission.gif', writer='imagemagick', fps=30)  #Requires imagemagick

plt.show()
