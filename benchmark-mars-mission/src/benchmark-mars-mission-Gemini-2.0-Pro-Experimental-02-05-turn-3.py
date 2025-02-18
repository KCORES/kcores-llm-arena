import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.integrate import solve_ivp

# --- Constants ---
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
M_sun = 1.989e30  # Mass of the Sun (kg)
M_earth = 5.972e24  # Mass of Earth (kg)
M_mars = 6.417e23  # Mass of Mars (kg)
AU = 1.496e11  # Astronomical unit (m) -  Earth's orbital radius
earth_orbital_period = 365.25 * 24 * 3600  # Earth's orbital period (s)
mars_orbital_period = 687 * 24 * 3600  # Mars' orbital period (s)
earth_orbital_radius = AU
mars_orbital_radius = 1.524 * AU  # Mars's orbital radius
earth_radius = 6.371e6
mars_radius = 3.3895e6
launch_window_interval = (
    780 * 24 * 3600
)  # Approximate synodic period (s) - Time between launch windows

# --- Hohmann Transfer Calculation ---


def hohmann_transfer(r1, r2):
    """Calculates the delta-v for a Hohmann transfer.

    Args:
        r1: Initial orbital radius.
        r2: Final orbital radius.

    Returns:
        A tuple: (delta_v1, delta_v2, transfer_time)
    """
    mu = G * M_sun  # Standard gravitational parameter of the Sun
    v1 = np.sqrt(mu / r1)
    v2 = np.sqrt(mu / r2)
    v_transfer_1 = np.sqrt(mu * (2 / r1 - 1 / ((r1 + r2) / 2)))
    v_transfer_2 = np.sqrt(mu * (2 / r2 - 1 / ((r1 + r2) / 2)))
    delta_v1 = abs(v_transfer_1 - v1)
    delta_v2 = abs(v_transfer_2 - v2)
    transfer_time = np.pi * np.sqrt(((r1 + r2) / 2) ** 3 / mu)  # in seconds
    return delta_v1, delta_v2, transfer_time


delta_v_earth, delta_v_mars, transfer_time = hohmann_transfer(
    earth_orbital_radius, mars_orbital_radius
)


# --- Orbital Positions (Simplified, circular orbits) ---


def earth_position(t):
    """Calculates Earth's position at time t."""
    angle = 2 * np.pi * t / earth_orbital_period
    return earth_orbital_radius * np.array([np.cos(angle), np.sin(angle), 0])


def mars_position(t, initial_mars_angle=0):  # Add initial angle
    """Calculates Mars' position at time t, accounting for initial position."""
    angle = 2 * np.pi * t / mars_orbital_period + initial_mars_angle
    return mars_orbital_radius * np.array([np.cos(angle), np.sin(angle), 0])


# --- Trajectory Simulation (Simplified, using Hohmann Transfer) ---


def trajectory(t0, transfer_time, launch_angle_mars=0):
    """Calculates the spacecraft's trajectory (simplified).

    Args:
      t0:  Start time of the simulation (launch time)
      transfer_time: Duration of the Hohmann transfer
      launch_angle_mars: Initial angular position of Mars (radians) at launch

    Returns:
      time_points: A list of time points.
      positions: A list of (x, y, z) positions of the spacecraft.
    """

    time_points = []
    positions = []

    # Phase 1: Earth Departure (Assume instantaneous delta-v)
    # We just add a small offset to the Earth's position for visual clarity.
    earth_pos = earth_position(t0)
    departure_offset = earth_pos * 0.02  # 2% of Earth's orbital radius
    time_points.append(t0)
    positions.append(earth_pos + departure_offset)

    # Phase 2: Hohmann Transfer
    num_transfer_points = 50  # Number of points to sample during the transfer
    transfer_times = np.linspace(t0, t0 + transfer_time, num_transfer_points)
    for t in transfer_times:
        frac = (t - t0) / transfer_time  # fraction of transfer completed
        r = earth_orbital_radius * (1 - frac) + mars_orbital_radius * frac

        # calculate weighted average angle
        earth_angle = 2 * np.pi * t0 / earth_orbital_period  # starting angle
        mars_angle = (
            2 * np.pi * (t0 + transfer_time) / mars_orbital_period + launch_angle_mars
        )
        current_angle = earth_angle * (1 - frac) + mars_angle * frac

        x = r * np.cos(current_angle)
        y = r * np.sin(current_angle)
        positions.append(np.array([x, y, 0]))
        time_points.append(t)
        # print(current_angle, x, y) #debug info.

    # Phase 3: Mars Arrival (Assume instantaneous delta-v)
    mars_pos = mars_position(t0 + transfer_time, launch_angle_mars)
    arrival_offset = mars_pos * 0.02  # 2% offset
    time_points.append(t0 + transfer_time)
    positions.append(mars_pos + arrival_offset)

    return time_points, positions


# --- Find Optimal Launch Window ---


def find_launch_window(start_time):
    """Finds the optimal launch time for a Hohmann transfer to Mars.

    This is a simplified version that assumes circular orbits and checks
    alignment within a tolerance.

    Args:
        start_time: The earliest time to start checking for launch windows.

    Returns:
       Optimal launch time.
    """

    best_launch_time = None
    min_angle_diff = float("inf")
    tolerance = 0.05  # radians, adjust as needed

    for t in np.arange(start_time, start_time + 2 * launch_window_interval, 86400):
        earth_pos = earth_position(t)
        mars_pos_at_arrival = mars_position(t + transfer_time)

        # Calculate the *angular* separation
        earth_angle = np.arctan2(earth_pos[1], earth_pos[0])
        mars_arrival_angle = np.arctan2(mars_pos_at_arrival[1], mars_pos_at_arrival[0])
        angle_diff = abs(
            mars_arrival_angle - earth_angle - np.pi
        )  # Ideal is 180 degrees (pi)
        angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # Take smaller angle

        if angle_diff < min_angle_diff:  # Find time with minimum angle diff
            min_angle_diff = angle_diff
            best_launch_time = t
            best_launch_angle_mars = np.arctan2(
                mars_position(t)[1], mars_position(t)[0]
            )  # Initial angle of Mars

        if angle_diff < tolerance:  # If within tolerance, return directly
            return t, np.arctan2(mars_position(t)[1], mars_position(t)[0])

    return (
        best_launch_time,
        best_launch_angle_mars,
    )  # Return best time found, even if not perfect


# --- Animation Setup ---

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Set limits (slightly larger than Mars orbit)
limit = mars_orbital_radius * 1.2
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-limit * 0.1, limit * 0.1)  # Flatten the z-axis for better visualization

# Sun (stationary)
ax.scatter(0, 0, 0, color="yellow", s=100, label="Sun")

# Earth and Mars (initial positions)
earth = ax.scatter([], [], [], color="blue", s=50, label="Earth")
mars = ax.scatter([], [], [], color="red", s=40, label="Mars")

# Spacecraft
(spacecraft,) = ax.plot(
    [], [], [], color="green", marker="o", markersize=4, label="Spacecraft"
)
(trajectory_line,) = ax.plot(
    [], [], [], color="gray", linestyle="dashed", linewidth=1
)  # Trajectory path

# --- Animation Function ---


def animate(
    frame_time,
    t0_earth_mars,
    transfer_time_earth_mars,
    t0_mars_earth,
    transfer_time_mars_earth,
    launch_angle_mars,
    launch_angle_earth,
):

    # --- Earth and Mars orbital motion ---
    earth_pos = earth_position(frame_time)
    earth.set_data([earth_pos[0]], [earth_pos[1]])
    earth.set_3d_properties([earth_pos[2]])

    mars_pos = mars_position(frame_time, launch_angle_mars)
    mars.set_data([mars_pos[0]], [mars_pos[1]])
    mars.set_3d_properties([mars_pos[2]])

    # --- Spacecraft Trajectory ---
    if frame_time <= t0_earth_mars + transfer_time_earth_mars:  # Earth to Mars
        if frame_time >= t0_earth_mars:
            times, positions = trajectory(
                t0_earth_mars, transfer_time_earth_mars, launch_angle_mars
            )
            # Find index of closest time
            idx = np.argmin(np.abs(np.array(times) - frame_time))
            spacecraft.set_data([positions[idx][0]], [positions[idx][1]])
            spacecraft.set_3d_properties([positions[idx][2]])
            trajectory_line.set_data(
                [p[0] for p in positions[: idx + 1]],
                [p[1] for p in positions[: idx + 1]],
            )
            trajectory_line.set_3d_properties([p[2] for p in positions[: idx + 1]])

    elif frame_time <= t0_mars_earth + transfer_time_mars_earth:  # Mars to Earth
        if frame_time >= t0_mars_earth:
            times, positions = trajectory(
                t0_mars_earth, transfer_time_mars_earth, launch_angle_earth
            )
            # Find index of closest time
            idx = np.argmin(np.abs(np.array(times) - frame_time))

            spacecraft.set_data([positions[idx][0]], [positions[idx][1]])
            spacecraft.set_3d_properties([positions[idx][2]])
            trajectory_line.set_data(
                [p[0] for p in positions[: idx + 1]],
                [p[1] for p in positions[: idx + 1]],
            )
            trajectory_line.set_3d_properties([p[2] for p in positions[: idx + 1]])
    else:
        # Hide spacecraft after the return
        spacecraft.set_data([], [])
        spacecraft.set_3d_properties([])
        trajectory_line.set_data([], [])
        trajectory_line.set_3d_properties([])

    ax.set_title(f"Time: {frame_time / (24 * 3600):.1f} days")  # Display time in days
    return earth, mars, spacecraft, trajectory_line


# --- Main Execution ---

if __name__ == "__main__":
    # 1. Find first launch window (Earth to Mars)
    start_time = 0
    t0_earth_mars, launch_angle_mars = find_launch_window(start_time)
    _, _, transfer_time_earth_mars = hohmann_transfer(
        earth_orbital_radius, mars_orbital_radius
    )

    # 2.  Stay on Mars (simplified - just a delay)
    stay_duration = 100 * 24 * 3600  # Stay on Mars for 100 days (example)

    # 3. Find second launch window (Mars to Earth)
    t0_mars_earth, launch_angle_earth = find_launch_window(
        t0_earth_mars + transfer_time_earth_mars + stay_duration
    )
    _, _, transfer_time_mars_earth = hohmann_transfer(
        mars_orbital_radius, earth_orbital_radius
    )  # Reverse transfer

    # 4.  Animation
    total_duration = (
        t0_mars_earth + transfer_time_mars_earth + 200 * 24 * 3600
    )  # Extra time for visualization
    frames = np.arange(
        0, total_duration, 0.5 * 24 * 3600
    )  # Time steps (e.g., every 0.5 days)

    ani = animation.FuncAnimation(
        fig,
        animate,
        frames=frames,
        fargs=(
            t0_earth_mars,
            transfer_time_earth_mars,
            t0_mars_earth,
            transfer_time_mars_earth,
            launch_angle_mars,
            launch_angle_earth,
        ),
        blit=True,
        repeat=False,
        interval=20,
    )

    ax.legend()
    plt.show()
