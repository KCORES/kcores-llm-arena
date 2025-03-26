import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# --- Constants ---
AU = 1.496e11  # Astronomical Unit in meters
SUN_RADIUS = 6.9634e8 / AU  # Sun radius in AU (for visualization scale)
EARTH_RADIUS_VIS = 0.03  # Visual radius for Earth in AU
MARS_RADIUS_VIS = 0.02  # Visual radius for Mars in AU
SPACECRAFT_SIZE = 5  # Marker size for spacecraft

# Orbital parameters (simplified: circular, coplanar)
EARTH_ORBIT_RADIUS = 1.0  # AU
MARS_ORBIT_RADIUS = 1.524  # AU
EARTH_PERIOD = 365.25  # days
MARS_PERIOD = 687.0  # days
EARTH_ANGULAR_VEL = 2 * np.pi / EARTH_PERIOD  # radians per day
MARS_ANGULAR_VEL = 2 * np.pi / MARS_PERIOD  # radians per day

# --- Mission Timing (Approximate) ---
# Start simulation slightly before launch
T_START_OFFSET = -50  # days before Earth launch
# Phase 1: Earth to Mars
T0_LAUNCH_EARTH = 0.0  # Day of launch from Earth
T1_ARRIVAL_MARS = 260.0  # Day of arrival at Mars (approx. travel time)
# Phase 2: Wait on Mars
T2_LAUNCH_MARS = (
    T1_ARRIVAL_MARS + 460.0
)  # Day of launch from Mars (approx. wait time for next window)
# Phase 3: Mars to Earth
T3_ARRIVAL_EARTH = (
    T2_LAUNCH_MARS + 260.0
)  # Day of arrival back at Earth (approx. travel time)
# End simulation slightly after arrival
T_END = T3_ARRIVAL_EARTH + 50  # days

# Simulation parameters
TIME_STEP = 2  # days per frame
TOTAL_DAYS = T_END - T_START_OFFSET
N_FRAMES = int(TOTAL_DAYS / TIME_STEP)
times = np.linspace(T_START_OFFSET, T_END, N_FRAMES)  # Array of time points

# --- Calculation Functions ---


def get_planet_pos(radius, angular_vel, time, initial_phase=0):
    """Calculates planet position at a given time."""
    angle = initial_phase + angular_vel * time
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0  # Coplanar orbits
    return np.array([x, y, z])


# --- Pre-calculate Positions ---

# Initial phase difference (Mars ahead of Earth for typical Hohmann)
# Mars angle = Earth angle + phase_diff
# Mars needs to be roughly 44 degrees ahead for E->M Hohmann.
initial_mars_phase = np.deg2rad(44.0)  # Mars starts ahead

earth_pos = np.array(
    [get_planet_pos(EARTH_ORBIT_RADIUS, EARTH_ANGULAR_VEL, t) for t in times]
)
mars_pos = np.array(
    [
        get_planet_pos(MARS_ORBIT_RADIUS, MARS_ANGULAR_VEL, t, initial_mars_phase)
        for t in times
    ]
)

# --- Calculate Spacecraft Trajectory (Simplified Arc Interpolation) ---
spacecraft_pos = np.zeros((N_FRAMES, 3))
craft_state = ["Pre-launch"] * N_FRAMES  # Track state for labeling

# Find indices corresponding to key mission times
idx0 = np.searchsorted(times, T0_LAUNCH_EARTH)
idx1 = np.searchsorted(times, T1_ARRIVAL_MARS)
idx2 = np.searchsorted(times, T2_LAUNCH_MARS)
idx3 = np.searchsorted(times, T3_ARRIVAL_EARTH)

# Get planet positions at key moments
pos_earth_launch = get_planet_pos(
    EARTH_ORBIT_RADIUS, EARTH_ANGULAR_VEL, T0_LAUNCH_EARTH
)
pos_mars_arrival = get_planet_pos(
    MARS_ORBIT_RADIUS, MARS_ANGULAR_VEL, T1_ARRIVAL_MARS, initial_mars_phase
)
pos_mars_launch = get_planet_pos(
    MARS_ORBIT_RADIUS, MARS_ANGULAR_VEL, T2_LAUNCH_MARS, initial_mars_phase
)
pos_earth_arrival = get_planet_pos(
    EARTH_ORBIT_RADIUS, EARTH_ANGULAR_VEL, T3_ARRIVAL_EARTH
)

for i, t in enumerate(times):
    if t < T0_LAUNCH_EARTH:
        # Phase 0: Pre-Launch - Spacecraft is on Earth
        spacecraft_pos[i] = earth_pos[i]
        craft_state[i] = "On Earth (Pre-Launch)"
    elif t < T1_ARRIVAL_MARS:
        # Phase 1: Earth to Mars transfer
        frac = (t - T0_LAUNCH_EARTH) / (T1_ARRIVAL_MARS - T0_LAUNCH_EARTH)
        # Simple interpolation: Use powers to create a curve (adjust exponent for more/less curve)
        # Interpolate radius and angle separately for a better arc
        r_start, r_end = EARTH_ORBIT_RADIUS, MARS_ORBIT_RADIUS
        angle_start = EARTH_ANGULAR_VEL * T0_LAUNCH_EARTH
        angle_end = MARS_ANGULAR_VEL * T1_ARRIVAL_MARS + initial_mars_phase

        # Handle angle wrapping carefully (shortest path)
        delta_angle = angle_end - angle_start
        delta_angle = (delta_angle + np.pi) % (
            2 * np.pi
        ) - np.pi  # Ensure shortest angle

        current_angle = angle_start + delta_angle * frac  # Simple linear angle interp
        current_radius = r_start + (r_end - r_start) * (
            np.sin(frac * np.pi / 2)
        )  # Non-linear radius interp

        spacecraft_pos[i, 0] = current_radius * np.cos(current_angle)
        spacecraft_pos[i, 1] = current_radius * np.sin(current_angle)
        spacecraft_pos[i, 2] = 0  # Keep in plane
        craft_state[i] = f"Transfer: Earth -> Mars ({frac*100:.0f}%)"

    elif t < T2_LAUNCH_MARS:
        # Phase 2: On Mars
        spacecraft_pos[i] = mars_pos[i]
        wait_frac = (t - T1_ARRIVAL_MARS) / (T2_LAUNCH_MARS - T1_ARRIVAL_MARS)
        craft_state[i] = f"On Mars (Waiting {wait_frac*100:.0f}%)"
    elif t < T3_ARRIVAL_EARTH:
        # Phase 3: Mars to Earth transfer
        frac = (t - T2_LAUNCH_MARS) / (T3_ARRIVAL_EARTH - T2_LAUNCH_MARS)
        # Interpolate radius and angle
        r_start, r_end = MARS_ORBIT_RADIUS, EARTH_ORBIT_RADIUS
        angle_start = MARS_ANGULAR_VEL * T2_LAUNCH_MARS + initial_mars_phase
        angle_end = EARTH_ANGULAR_VEL * T3_ARRIVAL_EARTH

        # Handle angle wrapping carefully (shortest path)
        delta_angle = angle_end - angle_start
        delta_angle = (delta_angle + np.pi) % (
            2 * np.pi
        ) - np.pi  # Ensure shortest angle

        current_angle = angle_start + delta_angle * frac  # Simple linear angle interp
        current_radius = r_start + (r_end - r_start) * (
            np.sin(frac * np.pi / 2)
        )  # Non-linear radius interp

        spacecraft_pos[i, 0] = current_radius * np.cos(current_angle)
        spacecraft_pos[i, 1] = current_radius * np.sin(current_angle)
        spacecraft_pos[i, 2] = 0  # Keep in plane
        craft_state[i] = f"Transfer: Mars -> Earth ({frac*100:.0f}%)"
    else:
        # Phase 4: Post-Arrival - Spacecraft is back on Earth
        spacecraft_pos[i] = earth_pos[i]
        craft_state[i] = "On Earth (Returned)"


# --- Plotting Setup ---
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot Sun
ax.scatter([0], [0], [0], s=100, color="yellow", label="Sun")

# Plot orbits (static)
earth_orbit_line = ax.plot(
    earth_pos[:, 0],
    earth_pos[:, 1],
    earth_pos[:, 2],
    color="blue",
    linestyle=":",
    linewidth=0.5,
    label="Earth Orbit",
)[0]
mars_orbit_line = ax.plot(
    mars_pos[:, 0],
    mars_pos[:, 1],
    mars_pos[:, 2],
    color="red",
    linestyle=":",
    linewidth=0.5,
    label="Mars Orbit",
)[0]

# Initialize dynamic plot elements
(earth_dot,) = ax.plot([], [], [], "o", color="blue", markersize=6, label="Earth")
(mars_dot,) = ax.plot([], [], [], "o", color="red", markersize=5, label="Mars")
(spacecraft_dot,) = ax.plot(
    [], [], [], "o", color="purple", markersize=SPACECRAFT_SIZE, label="Spacecraft"
)
(spacecraft_traj,) = ax.plot(
    [], [], [], "-", color="purple", linewidth=1, label="Trajectory"
)  # Trajectory line

# Add a text element for time and state
time_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, color="black")
state_text = ax.text2D(0.05, 0.90, "", transform=ax.transAxes, color="purple")


# Set plot limits and labels
ax.set_xlim([-MARS_ORBIT_RADIUS * 1.2, MARS_ORBIT_RADIUS * 1.2])
ax.set_ylim([-MARS_ORBIT_RADIUS * 1.2, MARS_ORBIT_RADIUS * 1.2])
ax.set_zlim(
    [-MARS_ORBIT_RADIUS * 0.5, MARS_ORBIT_RADIUS * 0.5]
)  # Smaller Z limit for better view
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars Round Trip Animation")
ax.legend(loc="upper right")
ax.set_aspect("equal", adjustable="box")  # Crucial for circular orbits to look circular
plt.grid(True)


# --- Animation Function ---
def update(frame):
    # Update planet positions
    earth_dot.set_data_3d(
        [earth_pos[frame, 0]], [earth_pos[frame, 1]], [earth_pos[frame, 2]]
    )
    mars_dot.set_data_3d(
        [mars_pos[frame, 0]], [mars_pos[frame, 1]], [mars_pos[frame, 2]]
    )

    # Update spacecraft position
    spacecraft_dot.set_data_3d(
        [spacecraft_pos[frame, 0]],
        [spacecraft_pos[frame, 1]],
        [spacecraft_pos[frame, 2]],
    )

    # Update spacecraft trajectory line (show path up to current frame)
    spacecraft_traj.set_data_3d(
        spacecraft_pos[: frame + 1, 0],
        spacecraft_pos[: frame + 1, 1],
        spacecraft_pos[: frame + 1, 2],
    )

    # Update text
    current_time = times[frame]
    time_text.set_text(f"Time: {current_time:.1f} days")
    state_text.set_text(f"Status: {craft_state[frame]}")

    # Return updated artists
    return earth_dot, mars_dot, spacecraft_dot, spacecraft_traj, time_text, state_text


# --- Create and Run Animation ---
ani = animation.FuncAnimation(
    fig,
    update,
    frames=N_FRAMES,
    interval=30,  # milliseconds per frame (adjust speed here)
    blit=True,  # Use blitting for performance
    repeat=True,
)  # Loop the animation

# To display the animation:
plt.show()

# To save the animation (requires ffmpeg or imagemagick):
# print("Saving animation... This may take a while.")
# ani.save('earth_mars_round_trip.gif', writer='imagemagick', fps=30)
# ani.save('earth_mars_round_trip.mp4', writer='ffmpeg', fps=30)
# print("Animation saved.")
