import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants and parameters (all distances in AU, time in days)
AU = 1.496e8  # km (not directly needed, for context)
earth_orbit_radius = 1.0  # AU
mars_orbit_radius = 1.524  # AU
sun_pos = np.array([0, 0, 0])

# Orbital periods (simplified circular orbits)
T_earth = 365.25  # days
T_mars = 686.98  # days

# Time parameters
total_days = int(
    2 * T_earth
)  # Simulate ~2 years (cover ~1 Earth-Mars launch window + return)
dt = 2  # days per frame


# Calculate positions of Earth and Mars in their orbits at time t (days)
def planet_position(orbit_radius, period, t):
    # Angular position in orbit (radians)
    theta = 2 * np.pi * (t % period) / period
    x = orbit_radius * np.cos(theta)
    y = orbit_radius * np.sin(theta)
    z = 0  # Assume orbits in ecliptic plane for simplicity
    return np.array([x, y, z])


# Hohmann Transfer characteristics (very simplified)
# Semi-major axis of transfer ellipse
a_transfer = (earth_orbit_radius + mars_orbit_radius) / 2

# Time of flight for transfer ellipse (half the ellipse period)
T_transfer = np.pi * np.sqrt(
    a_transfer**3
)  # Kepler's 3rd law (in years), convert to days
T_transfer_days = T_transfer * 365.25

# Phase timing
launch_day = 0
arrival_day = launch_day + T_transfer_days
stay_days = 500  # Days on Mars before return
return_launch_day = arrival_day + stay_days
return_arrival_day = return_launch_day + T_transfer_days

# Prepare figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth to Mars Mission Animation")

# Draw Sun
(sun,) = ax.plot([0], [0], [0], marker="o", markersize=15, color="yellow", label="Sun")

# Planet plots
(earth_plot,) = ax.plot([], [], [], "o", color="blue", label="Earth")
(mars_plot,) = ax.plot([], [], [], "o", color="red", label="Mars")

# Spacecraft plot
(sc_plot,) = ax.plot([], [], [], "o", color="green", label="Spacecraft")

# Trajectory plot (optional for visuals)
(traj_plot,) = ax.plot([], [], [], "g--", lw=1, alpha=0.5, label="Transfer Orbit")

# Legend
ax.legend()


def spacecraft_position(t):
    """
    Calculate spacecraft position at time t (days).
    Approximated as:
    - Earth orbit until launch
    - Transfer elliptical orbit from Earth to Mars during transfer
    - At Mars position during stay
    - Transfer elliptical orbit back from Mars to Earth
    - Back in Earth orbit after return
    """
    if t < launch_day:
        # Still on Earth orbit
        return planet_position(earth_orbit_radius, T_earth, t)
    elif launch_day <= t < arrival_day:
        # Outbound transfer from Earth to Mars
        # Parametrize transfer ellipse from 0 to pi
        transfer_t = (t - launch_day) / T_transfer_days * np.pi

        # Ellipse centered at focal point (Sun) located at origin,
        # with perihelion at Earth orbit, aphelion at Mars orbit.
        # Parametric equation of ellipse in x-y plane:
        # r = a(1 - e^2) / (1 + e*cos(theta))
        e = (mars_orbit_radius - earth_orbit_radius) / (
            mars_orbit_radius + earth_orbit_radius
        )
        a = a_transfer

        r = (
            a * (1 - e**2) / (1 + e * np.cos(transfer_t - np.pi))
        )  # shift by pi to start from Earth side
        x = r * np.cos(transfer_t)
        y = r * np.sin(transfer_t)
        z = 0
        return np.array([x, y, z])

    elif arrival_day <= t < return_launch_day:
        # Spacecraft is landed/stationary on Mars
        return planet_position(
            mars_orbit_radius, T_mars, arrival_day
        )  # fixed at Mars arrival position

    elif return_launch_day <= t < return_arrival_day:
        # Inbound transfer from Mars to Earth
        transfer_t = (t - return_launch_day) / T_transfer_days * np.pi

        # Ellipse parameters same, but travel opposite direction
        e = (mars_orbit_radius - earth_orbit_radius) / (
            mars_orbit_radius + earth_orbit_radius
        )
        a = a_transfer

        r = a * (1 - e**2) / (1 + e * np.cos(transfer_t))
        x = r * np.cos(np.pi + transfer_t)  # shifted by pi to start from Mars orbit
        y = r * np.sin(np.pi + transfer_t)
        z = 0
        return np.array([x, y, z])

    else:
        # Back on Earth orbit
        return planet_position(earth_orbit_radius, T_earth, t)


def init():
    earth_plot.set_data([], [])
    earth_plot.set_3d_properties([])

    mars_plot.set_data([], [])
    mars_plot.set_3d_properties([])

    sc_plot.set_data([], [])
    sc_plot.set_3d_properties([])

    traj_plot.set_data([], [])
    traj_plot.set_3d_properties([])

    return earth_plot, mars_plot, sc_plot, traj_plot


def update(frame):
    t = frame * dt

    # Update planet positions
    earth_pos = planet_position(earth_orbit_radius, T_earth, t)
    mars_pos = planet_position(mars_orbit_radius, T_mars, t)

    earth_plot.set_data(earth_pos[0], earth_pos[1])
    earth_plot.set_3d_properties(earth_pos[2])

    mars_plot.set_data(mars_pos[0], mars_pos[1])
    mars_plot.set_3d_properties(mars_pos[2])

    # Update spacecraft position
    sc_pos = spacecraft_position(t)
    sc_plot.set_data(sc_pos[0], sc_pos[1])
    sc_plot.set_3d_properties(sc_pos[2])

    # Update transfer trajectory (just show ellipse arcs - optional)
    num_points = 100

    # Outbound ellipse points
    e = (mars_orbit_radius - earth_orbit_radius) / (
        mars_orbit_radius + earth_orbit_radius
    )
    a = a_transfer

    theta_out = np.linspace(0, np.pi, num_points)
    r_out = a * (1 - e**2) / (1 + e * np.cos(theta_out - np.pi))
    x_out = r_out * np.cos(theta_out)
    y_out = r_out * np.sin(theta_out)

    # Inbound ellipse points
    theta_in = np.linspace(0, np.pi, num_points)
    r_in = a * (1 - e**2) / (1 + e * np.cos(theta_in))
    x_in = r_in * np.cos(np.pi + theta_in)
    y_in = r_in * np.sin(np.pi + theta_in)

    # Combine for visualization
    x = np.concatenate([x_out, x_in])
    y = np.concatenate([y_out, y_in])
    z = np.zeros_like(x)

    traj_plot.set_data(x, y)
    traj_plot.set_3d_properties(z)

    ax.set_title(
        f"Day: {int(t)} / {(int(total_days))} - Space Mission Earth-Mars-Earth"
    )

    return earth_plot, mars_plot, sc_plot, traj_plot


anim = FuncAnimation(
    fig, update, frames=int(total_days / dt), init_func=init, blit=True, interval=50
)

plt.show()
