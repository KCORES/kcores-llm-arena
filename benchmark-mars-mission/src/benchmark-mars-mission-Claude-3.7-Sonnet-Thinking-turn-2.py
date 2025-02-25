import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants (in AU, years, and Earth masses)
G = 4 * np.pi**2  # Gravitational constant in AU^3/year^2/solar_mass
sun_mass = 333000  # Sun mass in Earth masses

# Orbital parameters
earth_semi_major = 1.0  # AU
earth_period = 1.0  # years
earth_ecc = 0.0167

mars_semi_major = 1.524  # AU
mars_period = 1.88  # years
mars_ecc = 0.0934


def orbital_position(semi_major, ecc, period, time):
    """Calculate position in orbital plane at given time"""
    # Mean anomaly
    M = 2 * np.pi * time / period

    # Solve Kepler's equation iteratively for eccentric anomaly E
    E = M
    for i in range(10):  # Usually converges quickly
        E = M + ecc * np.sin(E)

    # True anomaly
    v = 2 * np.atan2(np.sqrt(1 + ecc) * np.sin(E / 2), np.sqrt(1 - ecc) * np.cos(E / 2))

    # Distance from focus
    r = semi_major * (1 - ecc * np.cos(E))

    # Position in orbital plane
    x = r * np.cos(v)
    y = r * np.sin(v)
    return x, y, 0  # z=0 for simple 2D orbits in 3D space


# Calculate Hohmann transfer orbit Earth to Mars
def hohmann_transfer(r1, r2, time_offset, time, duration):
    """Calculate transfer orbit position at given time"""
    a_transfer = (r1 + r2) / 2
    period_transfer = np.sqrt(a_transfer**3 / sun_mass) * 2 * np.pi

    # Transfer duration is half the period
    half_period = period_transfer / 2

    # Check if we're in the transfer window
    rel_time = time - time_offset
    if 0 <= rel_time <= half_period and rel_time <= duration:
        # Mean anomaly (starts at perihelion)
        M = 2 * np.pi * rel_time / period_transfer

        # Solve Kepler's equation for eccentric anomaly E
        E = M
        for i in range(10):
            E = M + (r2 - r1) / (r2 + r1) * np.sin(E)

        # Position on transfer ellipse
        v = 2 * np.atan2(np.sqrt((r2 + r1) / (r2 - r1)) * np.sin(E / 2), np.cos(E / 2))
        r = a_transfer * (1 - (r2 - r1) / (r2 + r1) * np.cos(E))

        x = r * np.cos(v)
        y = r * np.sin(v)
        return x, y, 0, True
    else:
        return 0, 0, 0, False


# Setup the figure and 3D axis
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection="3d")

# Time parameters for simulation
time_span = 4  # years
time_step = 0.01
times = np.arange(0, time_span, time_step)

# Starting positions - offset Mars by π/2
earth_offset = 0
mars_offset = np.pi / 2

# Launch windows (approximate)
earth_to_mars_launch = 0.3  # years
mars_to_earth_launch = 2.0  # years
transfer_duration = 0.8  # years

# Create the Sun
sun = ax.scatter([0], [0], [0], color="yellow", s=300, label="Sun")

# Initialize planet and spacecraft plots
earth_plot = ax.plot([], [], [], "bo", markersize=8, label="Earth")[0]
mars_plot = ax.plot([], [], [], "ro", markersize=6, label="Mars")[0]
spacecraft_plot = ax.plot([], [], [], "ko", markersize=4, label="Spacecraft")[0]

# Trajectories
earth_trajectory = ax.plot([], [], [], "b-", alpha=0.3)[0]
mars_trajectory = ax.plot([], [], [], "r-", alpha=0.3)[0]
spacecraft_trajectory = ax.plot([], [], [], "k-", alpha=0.5)[0]

# Storage for spacecraft trajectory
spacecraft_path = []


# Initialize function
def init():
    # Calculate and plot full orbits
    t_orbit = np.linspace(0, 2 * np.pi, 1000)

    # Earth orbit
    x_earth = earth_semi_major * np.cos(t_orbit)
    y_earth = earth_semi_major * np.sin(t_orbit)
    earth_trajectory.set_data(x_earth, y_earth)
    earth_trajectory.set_3d_properties(np.zeros_like(t_orbit))

    # Mars orbit
    x_mars = mars_semi_major * np.cos(t_orbit)
    y_mars = mars_semi_major * np.sin(t_orbit)
    mars_trajectory.set_data(x_mars, y_mars)
    mars_trajectory.set_3d_properties(np.zeros_like(t_orbit))

    # Set axis limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-0.5, 0.5)

    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Earth to Mars Round Trip Mission")

    return earth_trajectory, mars_trajectory, spacecraft_trajectory


# Animation update function
def update(frame):
    time = times[frame]

    # Calculate Earth and Mars positions
    x_earth, y_earth, z_earth = orbital_position(
        earth_semi_major, earth_ecc, earth_period, time + earth_offset
    )
    x_mars, y_mars, z_mars = orbital_position(
        mars_semi_major, mars_ecc, mars_period, time + mars_offset
    )

    # Update planet positions
    earth_plot.set_data([x_earth], [y_earth])
    earth_plot.set_3d_properties([z_earth])

    mars_plot.set_data([x_mars], [y_mars])
    mars_plot.set_3d_properties([z_mars])

    # Spacecraft logic - check if in transfer or with a planet
    if time < earth_to_mars_launch:
        # Spacecraft is on Earth
        x_sc, y_sc, z_sc = x_earth, y_earth, z_earth

    elif time < earth_to_mars_launch + transfer_duration:
        # First transfer: Earth to Mars
        earth_pos = orbital_position(
            earth_semi_major,
            earth_ecc,
            earth_period,
            earth_to_mars_launch + earth_offset,
        )
        mars_pos = orbital_position(
            mars_semi_major,
            mars_ecc,
            mars_period,
            earth_to_mars_launch + transfer_duration + mars_offset,
        )

        x_sc, y_sc, z_sc, _ = hohmann_transfer(
            earth_semi_major,
            mars_semi_major,
            earth_to_mars_launch,
            time,
            transfer_duration,
        )

    elif time < mars_to_earth_launch:
        # Spacecraft is on Mars
        x_sc, y_sc, z_sc = x_mars, y_mars, z_mars

    elif time < mars_to_earth_launch + transfer_duration:
        # Second transfer: Mars to Earth
        mars_pos = orbital_position(
            mars_semi_major, mars_ecc, mars_period, mars_to_earth_launch + mars_offset
        )
        earth_pos = orbital_position(
            earth_semi_major,
            earth_ecc,
            earth_period,
            mars_to_earth_launch + transfer_duration + earth_offset,
        )

        # Reverse direction for return journey
        x_sc, y_sc, z_sc, _ = hohmann_transfer(
            mars_semi_major,
            earth_semi_major,
            mars_to_earth_launch,
            time,
            transfer_duration,
        )

    else:
        # Back on Earth
        x_sc, y_sc, z_sc = x_earth, y_earth, z_earth

    # Update spacecraft position
    spacecraft_plot.set_data([x_sc], [y_sc])
    spacecraft_plot.set_3d_properties([z_sc])

    # Add point to spacecraft trajectory
    spacecraft_path.append((x_sc, y_sc, z_sc))
    if len(spacecraft_path) > 1:
        x_path, y_path, z_path = zip(*spacecraft_path)
        spacecraft_trajectory.set_data(x_path, y_path)
        spacecraft_trajectory.set_3d_properties(z_path)

    return earth_plot, mars_plot, spacecraft_plot, spacecraft_trajectory


# Create animation
ani = FuncAnimation(
    fig, update, frames=len(times), init_func=init, interval=20, blit=True
)

# Add a legend
ax.legend()

plt.show()

# Uncomment to save animation
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30)
