import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants (in Astronomical Units and Earth days)
AU = 149.6e6  # 1 AU in kilometers
EARTH_ORBIT_RADIUS = 1.0  # AU
MARS_ORBIT_RADIUS = 1.524  # AU
EARTH_PERIOD = 365.25  # days
MARS_PERIOD = 686.98  # days
SUN_RADIUS = 0.00465  # AU
EARTH_RADIUS = 0.00004263  # AU
MARS_RADIUS = 0.00002269  # AU

# Approximate time between launch windows
LAUNCH_WINDOW = 780  # days


def calculate_planet_position(radius, period, time):
    """Calculate position of a planet in its orbit at given time"""
    angular_velocity = 2 * np.pi / period
    x = radius * np.cos(angular_velocity * time)
    y = radius * np.sin(angular_velocity * time)
    z = 0
    return x, y, z


def calculate_transfer_orbit(r1, r2, mu, elapsed_time, total_transfer_time):
    """Calculate position on a Hohmann transfer orbit"""
    # Semi-major axis of transfer orbit
    a_transfer = (r1 + r2) / 2

    # We only want half of the elliptical orbit
    if elapsed_time <= total_transfer_time / 2:
        # Calculate angle along the elliptical orbit
        theta = np.pi * elapsed_time / (total_transfer_time / 2)

        # Calculate position along transfer ellipse
        a = a_transfer
        b = np.sqrt(r1 * r2)  # Semi-minor axis

        x = a * np.cos(theta)
        y = b * np.sin(theta)
        z = 0

        return x, y, z
    else:
        # Return None if we're past the transfer time
        return None, None, None


def create_sphere(radius, center_x, center_y, center_z):
    """Create a sphere for plotting"""
    u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
    x = radius * np.cos(u) * np.sin(v) + center_x
    y = radius * np.sin(u) * np.sin(v) + center_y
    z = radius * np.cos(v) + center_z
    return x, y, z


def main():
    # Create figure and 3D axis
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection="3d")

    # GM of the Sun (gravitational parameter)
    mu = 4 * np.pi**2  # AU^3/year^2 (in these units, G*M_sun = 4π²)

    # Calculate Hohmann transfer times (half orbit time)
    a_transfer = (EARTH_ORBIT_RADIUS + MARS_ORBIT_RADIUS) / 2
    transfer_time = np.pi * np.sqrt(a_transfer**3 / mu) * 365.25  # in days

    # Mission timeline (days)
    earth_departure = 0
    mars_arrival = earth_departure + transfer_time
    mars_departure = mars_arrival + LAUNCH_WINDOW - transfer_time - transfer_time
    earth_return = mars_departure + transfer_time

    # Total mission time with a bit extra
    total_time = earth_return + 50

    # Time points for animation
    num_frames = 200
    time_points = np.linspace(0, total_time, num_frames)

    # Draw the Sun
    x_sun, y_sun, z_sun = create_sphere(SUN_RADIUS, 0, 0, 0)
    ax.plot_surface(x_sun, y_sun, z_sun, color="yellow", alpha=0.7)

    # Draw planet orbits
    theta = np.linspace(0, 2 * np.pi, 100)
    earth_orbit_x = EARTH_ORBIT_RADIUS * np.cos(theta)
    earth_orbit_y = EARTH_ORBIT_RADIUS * np.sin(theta)
    earth_orbit_z = np.zeros_like(theta)

    mars_orbit_x = MARS_ORBIT_RADIUS * np.cos(theta)
    mars_orbit_y = MARS_ORBIT_RADIUS * np.sin(theta)
    mars_orbit_z = np.zeros_like(theta)

    ax.plot(
        earth_orbit_x,
        earth_orbit_y,
        earth_orbit_z,
        "b--",
        alpha=0.5,
        label="Earth Orbit",
    )
    ax.plot(
        mars_orbit_x, mars_orbit_y, mars_orbit_z, "r--", alpha=0.5, label="Mars Orbit"
    )

    # Lists to store positions for animation
    earth_positions = []
    mars_positions = []
    spacecraft_positions = []

    # Calculate planet and spacecraft positions at each time point
    for t in time_points:
        # Calculate Earth and Mars positions
        earth_x, earth_y, earth_z = calculate_planet_position(
            EARTH_ORBIT_RADIUS, EARTH_PERIOD, t
        )
        mars_x, mars_y, mars_z = calculate_planet_position(
            MARS_ORBIT_RADIUS, MARS_PERIOD, t
        )

        earth_positions.append((earth_x, earth_y, earth_z))
        mars_positions.append((mars_x, mars_y, mars_z))

        # Calculate spacecraft position based on mission phase
        if t < earth_departure:
            # Spacecraft is at Earth
            spacecraft_x, spacecraft_y, spacecraft_z = earth_x, earth_y, earth_z

        elif earth_departure <= t < mars_arrival:
            # Earth to Mars transfer
            # Calculate position on transfer orbit
            elapsed = t - earth_departure
            transfer_duration = mars_arrival - earth_departure

            # Get transfer orbit position
            x_tr, y_tr, z_tr = calculate_transfer_orbit(
                EARTH_ORBIT_RADIUS, MARS_ORBIT_RADIUS, mu, elapsed, transfer_duration
            )

            # Adjust rotation to align with departure point
            departure_angle = np.arctan2(earth_positions[0][1], earth_positions[0][0])

            # Rotate the transfer orbit
            spacecraft_x = x_tr * np.cos(departure_angle) - y_tr * np.sin(
                departure_angle
            )
            spacecraft_y = x_tr * np.sin(departure_angle) + y_tr * np.cos(
                departure_angle
            )
            spacecraft_z = z_tr

        elif mars_arrival <= t < mars_departure:
            # Spacecraft is at Mars
            spacecraft_x, spacecraft_y, spacecraft_z = mars_x, mars_y, mars_z

        elif mars_departure <= t < earth_return:
            # Mars to Earth transfer
            elapsed = t - mars_departure
            transfer_duration = earth_return - mars_departure

            # Get transfer orbit position
            x_tr, y_tr, z_tr = calculate_transfer_orbit(
                MARS_ORBIT_RADIUS, EARTH_ORBIT_RADIUS, mu, elapsed, transfer_duration
            )

            # Adjust rotation to align with departure point
            departure_angle = np.arctan2(
                mars_positions[num_frames // 3][1], mars_positions[num_frames // 3][0]
            )

            # Rotate the transfer orbit
            spacecraft_x = x_tr * np.cos(departure_angle) - y_tr * np.sin(
                departure_angle
            )
            spacecraft_y = x_tr * np.sin(departure_angle) + y_tr * np.cos(
                departure_angle
            )
            spacecraft_z = z_tr

        else:
            # Spacecraft is back at Earth
            spacecraft_x, spacecraft_y, spacecraft_z = earth_x, earth_y, earth_z

        spacecraft_positions.append((spacecraft_x, spacecraft_y, spacecraft_z))

    # Create initial objects for animation
    earth_x0, earth_y0, earth_z0 = earth_positions[0]
    mars_x0, mars_y0, mars_z0 = mars_positions[0]
    spacecraft_x0, spacecraft_y0, spacecraft_z0 = spacecraft_positions[0]

    # Plot Earth
    x_earth, y_earth, z_earth = create_sphere(
        EARTH_RADIUS, earth_x0, earth_y0, earth_z0
    )
    earth_surface = ax.plot_surface(x_earth, y_earth, z_earth, color="blue", alpha=0.7)

    # Plot Mars
    x_mars, y_mars, z_mars = create_sphere(MARS_RADIUS, mars_x0, mars_y0, mars_z0)
    mars_surface = ax.plot_surface(x_mars, y_mars, z_mars, color="red", alpha=0.7)

    # Plot spacecraft
    (spacecraft,) = ax.plot(
        [spacecraft_x0],
        [spacecraft_y0],
        [spacecraft_z0],
        "ko",
        ms=5,
        label="Spacecraft",
    )

    # Plot trajectory line (initially empty)
    (trajectory,) = ax.plot([], [], [], "k-", alpha=0.5)
    trajectory_x, trajectory_y, trajectory_z = [], [], []

    # Set plot limits
    max_orbit = max(EARTH_ORBIT_RADIUS, MARS_ORBIT_RADIUS)
    ax.set_xlim(-max_orbit * 1.2, max_orbit * 1.2)
    ax.set_ylim(-max_orbit * 1.2, max_orbit * 1.2)
    ax.set_zlim(-max_orbit * 0.5, max_orbit * 0.5)

    # Add labels and title
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Earth to Mars Mission Trajectory")
    ax.legend()

    # Text annotations
    time_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)
    phase_text = ax.text2D(0.05, 0.9, "", transform=ax.transAxes)

    # Update function for animation
    def update(frame):
        # Current time
        t = time_points[frame]

        # Update Earth position
        earth_x, earth_y, earth_z = earth_positions[frame]
        x_earth, y_earth, z_earth = create_sphere(
            EARTH_RADIUS, earth_x, earth_y, earth_z
        )

        # Remove old Earth and plot new one
        earth_surface.remove()
        earth_surface = ax.plot_surface(
            x_earth, y_earth, z_earth, color="blue", alpha=0.7
        )

        # Update Mars position
        mars_x, mars_y, mars_z = mars_positions[frame]
        x_mars, y_mars, z_mars = create_sphere(MARS_RADIUS, mars_x, mars_y, mars_z)

        # Remove old Mars and plot new one
        mars_surface.remove()
        mars_surface = ax.plot_surface(x_mars, y_mars, z_mars, color="red", alpha=0.7)

        # Update spacecraft position
        spacecraft_x, spacecraft_y, spacecraft_z = spacecraft_positions[frame]
        spacecraft.set_data([spacecraft_x], [spacecraft_y])
        spacecraft.set_3d_properties([spacecraft_z])

        # Update trajectory line
        trajectory_x.append(spacecraft_x)
        trajectory_y.append(spacecraft_y)
        trajectory_z.append(spacecraft_z)
        trajectory.set_data(trajectory_x, trajectory_y)
        trajectory.set_3d_properties(trajectory_z)

        # Update time text
        time_text.set_text(f"Mission Time: {t:.1f} days")

        # Update mission phase text
        if t < earth_departure:
            phase = "Waiting for launch"
        elif earth_departure <= t < mars_arrival:
            phase = "Earth to Mars transfer"
        elif mars_arrival <= t < mars_departure:
            phase = "At Mars, waiting for return window"
        elif mars_departure <= t < earth_return:
            phase = "Mars to Earth transfer"
        else:
            phase = "Returned to Earth"
        phase_text.set_text(f"Mission Phase: {phase}")

        return (
            earth_surface,
            mars_surface,
            spacecraft,
            trajectory,
            time_text,
            phase_text,
        )

    # Create animation
    ani = FuncAnimation(
        fig, update, frames=range(len(time_points)), interval=50, blit=False
    )

    plt.tight_layout()
    plt.show()

    # Uncomment to save the animation
    # ani.save('mars_mission.mp4', writer='ffmpeg', dpi=200)


if __name__ == "__main__":
    main()
