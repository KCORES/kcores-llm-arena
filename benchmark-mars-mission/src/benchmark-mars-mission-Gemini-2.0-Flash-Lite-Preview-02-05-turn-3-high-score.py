import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

# --- Constants and Orbital Parameters ---

# Astronomical Unit (AU) in meters
AU = 149597870700  # m

# Gravitational Constant (m^3 kg^-1 s^-2)
G = 6.6743e-11

# Earth
M_earth = 5.972e24  # kg
R_earth = 6371000  # m
orbital_period_earth = 365.25 * 24 * 3600  # seconds
semi_major_axis_earth = 1 * AU  # AU
eccentricity_earth = 0.0167  # Small eccentricity

# Mars
M_mars = 6.417e23  # kg
R_mars = 3389500  # m
orbital_period_mars = 687 * 24 * 3600  # seconds
semi_major_axis_mars = 1.524 * AU  # AU
eccentricity_mars = 0.0934  # Eccentricity of Mars' orbit

# --- Helper Functions for Orbital Mechanics ---


def get_orbital_position(time, semi_major_axis, eccentricity, orbital_period):
    """
    Calculates the position of a planet in its orbit using the orbital parameters.
    Simplified:  Doesn't handle true anomaly and just uses a circular orbit for initial animation
    More accurate trajectory needs to solve the full 2-body or N-body problem.
    """
    angular_speed = 2 * np.pi / orbital_period
    mean_anomaly = angular_speed * time  # Mean anomaly

    # For a circular orbit (eccentricity = 0)
    x = semi_major_axis * np.cos(mean_anomaly)
    y = semi_major_axis * np.sin(mean_anomaly)
    return x, y


def calculate_transfer_orbit(
    t_start, t_end, earth_pos_init, earth_vel_init, mars_pos_init
):
    """
    Calculates the transfer orbit parameters (simplified).  Needs a more accurate model for full trajectory
    Uses Lambert's problem for the general case, but here simplifies to initial and final conditions
    """
    #  This is a *very* simplified calculation, assuming a Hohmann transfer.
    #  A proper trajectory calculation would require a Lambert solver.
    #  Also ignoring plane changes

    #  Simplified Transfer orbit parameters
    semi_major_axis_transfer = 0.5 * (earth_pos_init + mars_pos_init)  # Approx.

    return semi_major_axis_transfer


def solve_two_body_problem(time_span, initial_conditions, mu):
    """
    Solves the two-body problem using solve_ivp.
    Returns the position and velocity of the spacecraft.
    """

    def two_body_ode(t, state):
        rx, ry, rz, vx, vy, vz = state
        r = np.array([rx, ry, rz])
        r_mag = np.linalg.norm(r)
        acceleration = -mu * r / r_mag**3
        return [vx, vy, vz, acceleration[0], acceleration[1], acceleration[2]]

    solution = solve_ivp(two_body_ode, time_span, initial_conditions, dense_output=True)
    return solution.t, solution.sol(solution.t)


# --- Simulation Setup ---

# Animation Parameters
num_frames = 500
dt = 3600 * 10  # Time step in seconds for the animation
animation_speed_factor = 1  # Adjust this to control animation speed

# Launch Window (Simplified)
launch_window_start_time = 0  # seconds
launch_window_duration = 600  # seconds (arbitrary, very short in this example)
time_to_mars = 250 * 24 * 3600  # time to Mars in seconds (approximate for Hohmann)
time_to_earth = 250 * 24 * 3600  # time to Earth (approximate)

# --- Initial Conditions ---
# Note:  This will create a figure for a circular orbit approximation only.
# Real orbital mechanics is much more complex.

# Earth and Mars initial positions
earth_start_time = launch_window_start_time  # Start at time 0
mars_start_time = (
    launch_window_start_time + time_to_mars
)  # Time when Mars is at the correct position
earth_pos_init = np.array(
    [
        get_orbital_position(
            earth_start_time,
            semi_major_axis_earth,
            eccentricity_earth,
            orbital_period_earth,
        )[0],
        get_orbital_position(
            earth_start_time,
            semi_major_axis_earth,
            eccentricity_earth,
            orbital_period_earth,
        )[1],
        0,
    ]
)  # Z=0 to start simple
mars_pos_init = np.array(
    [
        get_orbital_position(
            mars_start_time,
            semi_major_axis_mars,
            eccentricity_mars,
            orbital_period_mars,
        )[0],
        get_orbital_position(
            mars_start_time,
            semi_major_axis_mars,
            eccentricity_mars,
            orbital_period_mars,
        )[1],
        0,
    ]
)
# Spacecraft initial conditions (Earth launch)
spacecraft_pos_init = earth_pos_init  # Assumes launch from Earth position
#  Simplified Hohmann transfer velocities
#  Would need Lambert's Solver for proper velocity calculation
mu_sun = G * (
    M_earth + M_mars
)  # Using combined mass since the transfer orbit isn't very accurate
#  Roughly estimate the initial velocity
v_earth = np.array([0, 0, 0])  # needs a proper orbit calculation
spacecraft_vel_init = np.array([0, 0, 0])

# --- Generate the full trajectory using a Two-Body Solver  ---

# Time span for the journey to Mars
t_to_mars = np.linspace(0, time_to_mars, num_frames)
initial_state_to_mars = [
    spacecraft_pos_init[0],
    spacecraft_pos_init[1],
    spacecraft_pos_init[2],
    spacecraft_vel_init[0],
    spacecraft_vel_init[1],
    spacecraft_vel_init[2],
]
t_mars_trajectory, y_mars_trajectory = solve_two_body_problem(
    (0, time_to_mars), initial_state_to_mars, mu_sun
)

# Mars orbit
mars_arrival_pos = np.array(
    [y_mars_trajectory[0][-1], y_mars_trajectory[1][-1], y_mars_trajectory[2][-1]]
)  # Position at Mars arrival
# Calculate the velocity after the Mars Orbit Insertion (MOI) maneuver
v_mars = [0, 0, 0]  # needs proper calculation.  Assume MOI and landing immediately
# ---  Trip Back to Earth ---
t_to_earth = np.linspace(
    0, time_to_earth, num_frames
)  # Use same number of frames for animation consistency
initial_state_to_earth = [
    mars_arrival_pos[0],
    mars_arrival_pos[1],
    mars_arrival_pos[2],
    v_mars[0],
    v_mars[1],
    v_mars[2],
]  # Assume MOI and launch immediately
t_earth_trajectory, y_earth_trajectory = solve_two_body_problem(
    (0, time_to_earth), initial_state_to_earth, mu_sun
)

# --- Animation Setup ---

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot Earth and Mars orbits (simplified circular orbits)
earth_orbit_x = [
    get_orbital_position(
        t, semi_major_axis_earth, eccentricity_earth, orbital_period_earth
    )[0]
    for t in np.linspace(0, orbital_period_earth, 1000)
]
earth_orbit_y = [
    get_orbital_position(
        t, semi_major_axis_earth, eccentricity_earth, orbital_period_earth
    )[1]
    for t in np.linspace(0, orbital_period_earth, 1000)
]
mars_orbit_x = [
    get_orbital_position(
        t, semi_major_axis_mars, eccentricity_mars, orbital_period_mars
    )[0]
    for t in np.linspace(0, orbital_period_mars, 1000)
]
mars_orbit_y = [
    get_orbital_position(
        t, semi_major_axis_mars, eccentricity_mars, orbital_period_mars
    )[1]
    for t in np.linspace(0, orbital_period_mars, 1000)
]
ax.plot(
    earth_orbit_x,
    earth_orbit_y,
    np.zeros_like(earth_orbit_x),
    "b--",
    label="Earth Orbit",
)
ax.plot(
    mars_orbit_x, mars_orbit_y, np.zeros_like(mars_orbit_x), "r--", label="Mars Orbit"
)

# Plot Sun
ax.scatter(0, 0, 0, c="yellow", s=100, label="Sun")

# Plot Earth and Mars Initial positions
(earth_plot,) = ax.plot([], [], [], "g.", markersize=10, label="Earth")
(mars_plot,) = ax.plot([], [], [], "m.", markersize=10, label="Mars")
(spacecraft_plot,) = ax.plot(
    [], [], [], "c-", linewidth=2, label="Spacecraft"
)  # Spacecraft trajectory

# Set axis limits (adjust as needed) - make sure it shows the orbits!
max_range = max(semi_major_axis_earth, semi_major_axis_mars) * 1.2
ax.set_xlim([-max_range, max_range])
ax.set_ylim([-max_range, max_range])
ax.set_zlim([-max_range * 0.2, max_range * 0.2])  # Keep the z-axis visible
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Earth to Mars and Back")
ax.legend()
ax.view_init(elev=20, azim=45)  # Adjust the view


# --- Animation Update Function ---
def update(frame):
    """
    Updates the plot for each frame of the animation.
    """
    current_time_mars = frame * dt * animation_speed_factor
    current_time_earth = (
        current_time_mars - time_to_mars
    )  # For time offset to the journey back

    # Get Earth and Mars positions at this time
    earth_x, earth_y = get_orbital_position(
        current_time_mars,
        semi_major_axis_earth,
        eccentricity_earth,
        orbital_period_earth,
    )
    mars_x, mars_y = get_orbital_position(
        current_time_mars - time_to_mars,
        semi_major_axis_mars,
        eccentricity_mars,
        orbital_period_mars,
    )  # time shifted

    # Update planet positions
    earth_plot.set_data([earth_x], [earth_y])
    earth_plot.set_3d_properties([0])  # Z = 0 plane
    mars_plot.set_data([mars_x], [mars_y])
    mars_plot.set_3d_properties([0])

    # --- Spacecraft Trajectory Update ---
    frame_to_mars = min(frame, len(t_mars_trajectory) - 1)
    spacecraft_trajectory_x = y_mars_trajectory[0][:frame_to_mars]
    spacecraft_trajectory_y = y_mars_trajectory[1][:frame_to_mars]
    spacecraft_trajectory_z = y_mars_trajectory[2][:frame_to_mars]

    #  Add the return trip
    frame_to_earth = max(0, frame - len(t_mars_trajectory))
    if frame >= len(t_mars_trajectory):
        spacecraft_trajectory_x = np.concatenate(
            (spacecraft_trajectory_x, y_earth_trajectory[0][:frame_to_earth])
        )
        spacecraft_trajectory_y = np.concatenate(
            (spacecraft_trajectory_y, y_earth_trajectory[1][:frame_to_earth])
        )
        spacecraft_trajectory_z = np.concatenate(
            (spacecraft_trajectory_z, y_earth_trajectory[2][:frame_to_earth])
        )
    spacecraft_plot.set_data(spacecraft_trajectory_x, spacecraft_trajectory_y)
    spacecraft_plot.set_3d_properties(spacecraft_trajectory_z)

    return earth_plot, mars_plot, spacecraft_plot


# --- Create and Run the Animation ---
ani = FuncAnimation(
    fig, update, frames=num_frames * 2, blit=False, repeat=False
)  # Run twice the number of frames

plt.show()
