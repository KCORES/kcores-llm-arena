import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# --- Constants ---
AU = 1.496e11  # Astronomical Unit in meters (used for scaling if needed, but we'll use AU directly)
SUN_POS = np.array([0.0, 0.0, 0.0])

# Orbital parameters (simplified: circular orbits in AU and days)
EARTH_RADIUS_ORBIT = 1.0  # AU
EARTH_PERIOD = 365.25  # Days
EARTH_ANGULAR_VEL = 2 * np.pi / EARTH_PERIOD  # Radians per day

MARS_RADIUS_ORBIT = 1.524  # AU
MARS_PERIOD = 687.0  # Days
MARS_ANGULAR_VEL = 2 * np.pi / MARS_PERIOD  # Radians per day

# --- Hohmann Transfer Calculations (Simplified) ---
# Earth to Mars
mu_sun = 1  # Standard gravitational parameter of the Sun (can be set to 1 if using AU and years, adjust if using days)
# Recalculate mu_sun for AU and days: G*M_sun. (2pi)^2 * a^3 / T^2 = G*M => mu = (2pi)^2 * a^3 / T^2
# For Earth: mu = (2*np.pi)**2 * EARTH_RADIUS_ORBIT**3 / (EARTH_PERIOD)**2
mu_sun = (2 * np.pi) ** 2 * EARTH_RADIUS_ORBIT**3 / EARTH_PERIOD**2

a_trans1 = (
    EARTH_RADIUS_ORBIT + MARS_RADIUS_ORBIT
) / 2.0  # Semi-major axis for E-M transfer
T_trans1_full = (
    2 * np.pi * np.sqrt(a_trans1**3 / mu_sun)
)  # Period of full transfer ellipse
TRANSFER_TIME_E_M = T_trans1_full / 2.0  # Hohmann transfer time E-M in days

# Mars to Earth
a_trans2 = a_trans1  # Same ellipse geometry for M-E transfer
T_trans2_full = T_trans1_full
TRANSFER_TIME_M_E = T_trans2_full / 2.0  # Hohmann transfer time M-E in days

print(f"Simplified Hohmann Transfer Times:")
print(f"  Earth -> Mars: {TRANSFER_TIME_E_M:.1f} days")
print(f"  Mars -> Earth: {TRANSFER_TIME_M_E:.1f} days")

# --- Phase Angles for Launch Windows ---
# Earth to Mars: Mars must be ahead of Earth
# Angle Mars travels during transfer: alpha_mars = MARS_ANGULAR_VEL * TRANSFER_TIME_E_M
# Angle spacecraft travels: pi
# Required phase angle (Mars ahead of Earth): phi = pi - alpha_mars
phase_angle_E_M = np.pi - MARS_ANGULAR_VEL * TRANSFER_TIME_E_M
print(
    f"Required Phase Angle (E->M): Mars ahead by {np.degrees(phase_angle_E_M):.1f} deg"
)

# Mars to Earth: Earth must be ahead of Mars (target position)
# Angle Earth travels during transfer: alpha_earth = EARTH_ANGULAR_VEL * TRANSFER_TIME_M_E
# Required phase angle (Earth relative to Mars at Mars departure): phi = pi - alpha_earth
# Often expressed as how far Earth is from opposition: Earth needs to be at angle alpha_earth when spacecraft arrives.
# Spacecraft travels pi. Mars is at departure angle theta_M. Earth must be at theta_M + pi - alpha_earth.
# Phase angle = (theta_M + pi - alpha_earth) - theta_M = pi - alpha_earth (relative to Mars's position)
# Alternative view: Where does Earth need to be *relative to Mars* when *leaving* Mars?
# Earth travels faster. When spacecraft arrives at Earth's orbit (pi radians later), Earth must be there.
# During transfer time T_M_E, Earth moves angle_E = EARTH_ANGULAR_VEL * T_M_E.
# The spacecraft arrives pi radians ahead of Mars' departure point.
# So, at Mars departure, Earth must be at an angle of (pi - angle_E) *behind* the Mars departure point.
# Or, equivalently, Mars must be angle_E - pi ahead of Earth. (This angle is > 180 deg usually)
# Let's stick to the convention: target leads source. Earth leads Mars.
phase_angle_M_E = np.pi - EARTH_ANGULAR_VEL * TRANSFER_TIME_M_E
print(
    f"Required Phase Angle (M->E): Earth ahead by {np.degrees(phase_angle_M_E):.1f} deg (relative target leads source)"
)


# --- Simulation Setup ---
TIME_STEP = 2.0  # Days per frame
TOTAL_YEARS = 4  # Approximate duration to find windows and complete trip
N_STEPS = int(TOTAL_YEARS * 365.25 / TIME_STEP)

# Initial positions (arbitrary start, e.g., Earth at 0 deg)
t0 = 0.0
earth_angle0 = 0.0
mars_angle0 = np.pi / 4  # Start Mars somewhere ahead

# --- Calculate Planetary Positions over Time ---
times = np.linspace(t0, t0 + TOTAL_YEARS * 365.25, N_STEPS)
earth_angles = earth_angle0 + EARTH_ANGULAR_VEL * times
mars_angles = mars_angle0 + MARS_ANGULAR_VEL * times

earth_x = EARTH_RADIUS_ORBIT * np.cos(earth_angles)
earth_y = EARTH_RADIUS_ORBIT * np.sin(earth_angles)
earth_z = np.zeros(N_STEPS)

mars_x = MARS_RADIUS_ORBIT * np.cos(mars_angles)
mars_y = MARS_RADIUS_ORBIT * np.sin(mars_angles)
mars_z = np.zeros(N_STEPS)

# --- Find Launch Windows and Calculate Trajectory ---
launch_time_1 = -1
arrival_time_mars = -1
launch_idx_1 = -1
arrival_idx_mars = -1

# Find first E-M launch window
print("\nSearching for Earth -> Mars launch window...")
for i in range(N_STEPS):
    current_time = times[i]
    # Angle difference: mars_angle - earth_angle (handle wrap around 2pi)
    delta_angle = (mars_angles[i] - earth_angles[i]) % (2 * np.pi)
    # Check if delta_angle is close to the required phase_angle_E_M
    # Tolerance can be adjusted
    if (
        abs(delta_angle - phase_angle_E_M) < EARTH_ANGULAR_VEL * TIME_STEP * 1.5
    ):  # Tolerate 1.5 steps error
        launch_time_1 = current_time
        launch_idx_1 = i
        arrival_time_mars = launch_time_1 + TRANSFER_TIME_E_M
        # Find the index corresponding to arrival time
        arrival_idx_mars = np.searchsorted(times, arrival_time_mars)
        if arrival_idx_mars >= N_STEPS:
            print("Error: Arrival time exceeds simulation duration.")
            arrival_idx_mars = N_STEPS - 1  # Cap index
            arrival_time_mars = times[arrival_idx_mars]  # Adjust arrival time
        print(
            f"  Found E->M launch window at Day {launch_time_1:.1f} (Index {launch_idx_1})"
        )
        print(
            f"  Estimated arrival at Mars on Day {arrival_time_mars:.1f} (Index {arrival_idx_mars})"
        )
        break
else:
    print("  E->M launch window not found within simulation time.")
    exit()  # Stop if no launch window

# Find next M-E launch window (after waiting on Mars)
launch_time_2 = -1
arrival_time_earth = -1
launch_idx_2 = -1
arrival_idx_earth = -1

print("\nSearching for Mars -> Earth launch window (after Mars arrival)...")
# Start searching from Mars arrival time
search_start_idx = arrival_idx_mars
if search_start_idx < 0:
    search_start_idx = 0  # Ensure non-negative index

for i in range(search_start_idx, N_STEPS):
    current_time = times[i]
    # Angle difference: earth_angle - mars_angle (handle wrap around 2pi)
    # We want Earth to be ahead by phase_angle_M_E
    delta_angle = (earth_angles[i] - mars_angles[i]) % (2 * np.pi)

    # Check if delta_angle is close to the required phase_angle_M_E
    if (
        abs(delta_angle - phase_angle_M_E) < MARS_ANGULAR_VEL * TIME_STEP * 1.5
    ):  # Tolerate 1.5 steps (Mars moves slower)
        launch_time_2 = current_time
        launch_idx_2 = i
        arrival_time_earth = launch_time_2 + TRANSFER_TIME_M_E
        arrival_idx_earth = np.searchsorted(times, arrival_time_earth)

        if arrival_idx_earth >= N_STEPS:
            print(
                "Warning: M->E arrival time might exceed simulation duration. Capping."
            )
            arrival_idx_earth = N_STEPS - 1  # Cap index
            arrival_time_earth = times[arrival_idx_earth]  # Adjust arrival time

        # Ensure sufficient wait time on Mars (synodic period ~780 days)
        wait_time = launch_time_2 - arrival_time_mars
        if wait_time < 30:  # Minimum stay sanity check (days)
            print(
                f"  Skipping potential M->E window at Day {launch_time_2:.1f} (Wait time {wait_time:.1f} days too short)."
            )
            continue  # Keep searching for a later window

        print(
            f"  Found M->E launch window at Day {launch_time_2:.1f} (Index {launch_idx_2})"
        )
        print(f"  Wait time on Mars: {wait_time:.1f} days")
        print(
            f"  Estimated arrival at Earth on Day {arrival_time_earth:.1f} (Index {arrival_idx_earth})"
        )
        break
else:
    print("  M->E launch window not found within simulation time after Mars arrival.")
    # Decide how to handle this: maybe extend simulation or just end animation early
    # For now, we'll let the animation run but the spacecraft might not return


# --- Calculate Spacecraft Path ---
sc_x = np.zeros(N_STEPS)
sc_y = np.zeros(N_STEPS)
sc_z = np.zeros(N_STEPS)
sc_state = ["Pre-launch"] * N_STEPS  # Track spacecraft state

# Phase 1: Earth Orbit (before launch 1)
for i in range(launch_idx_1):
    sc_x[i] = earth_x[i]
    sc_y[i] = earth_y[i]
    sc_z[i] = earth_z[i]
    sc_state[i] = "On Earth Orbit"

# Phase 2: Earth -> Mars Transfer
if launch_idx_1 != -1 and arrival_idx_mars != -1 and launch_idx_1 < arrival_idx_mars:
    start_pos = np.array(
        [earth_x[launch_idx_1], earth_y[launch_idx_1], earth_z[launch_idx_1]]
    )
    # Target position: Where Mars WILL BE at arrival time
    end_pos = np.array(
        [mars_x[arrival_idx_mars], mars_y[arrival_idx_mars], mars_z[arrival_idx_mars]]
    )
    transfer_indices = np.arange(
        launch_idx_1, arrival_idx_mars + 1
    )  # Indices during transfer

    # Linear interpolation for simplicity for visualization
    for k, idx in enumerate(transfer_indices):
        if idx >= N_STEPS:
            break  # Safety break
        frac = k / (len(transfer_indices) - 1) if len(transfer_indices) > 1 else 1.0
        interp_pos = start_pos + frac * (end_pos - start_pos)
        sc_x[idx] = interp_pos[0]
        sc_y[idx] = interp_pos[1]
        sc_z[idx] = interp_pos[2]  # Stays 0 in this model
        sc_state[idx] = "Transfer E->M"


# Phase 3: Waiting on Mars
wait_start_idx = arrival_idx_mars + 1
wait_end_idx = (
    launch_idx_2 if launch_idx_2 != -1 else N_STEPS
)  # Wait until launch 2 or end of sim

if launch_idx_1 != -1:  # Make sure first leg happened
    for i in range(wait_start_idx, wait_end_idx):
        if i >= N_STEPS:
            break  # Safety break
        sc_x[i] = mars_x[i]
        sc_y[i] = mars_y[i]
        sc_z[i] = mars_z[i]
        sc_state[i] = "On Mars Orbit"


# Phase 4: Mars -> Earth Transfer
if launch_idx_2 != -1 and arrival_idx_earth != -1 and launch_idx_2 < arrival_idx_earth:
    start_pos_2 = np.array(
        [mars_x[launch_idx_2], mars_y[launch_idx_2], mars_z[launch_idx_2]]
    )
    # Target position: Where Earth WILL BE at arrival time
    end_pos_2 = np.array(
        [
            earth_x[arrival_idx_earth],
            earth_y[arrival_idx_earth],
            earth_z[arrival_idx_earth],
        ]
    )
    transfer_indices_2 = np.arange(
        launch_idx_2, arrival_idx_earth + 1
    )  # Indices during transfer

    # Linear interpolation
    for k, idx in enumerate(transfer_indices_2):
        if idx >= N_STEPS:
            break  # Safety break
        # Avoid division by zero if indices are the same
        if len(transfer_indices_2) > 1:
            frac = k / (len(transfer_indices_2) - 1)
        else:
            frac = 1.0
        interp_pos_2 = start_pos_2 + frac * (end_pos_2 - start_pos_2)
        sc_x[idx] = interp_pos_2[0]
        sc_y[idx] = interp_pos_2[1]
        sc_z[idx] = interp_pos_2[2]  # Stays 0
        sc_state[idx] = "Transfer M->E"

# Phase 5: Back on Earth Orbit (after arrival 2)
if arrival_idx_earth != -1:
    for i in range(arrival_idx_earth + 1, N_STEPS):
        if i >= N_STEPS:
            break  # Safety break
        sc_x[i] = earth_x[i]
        sc_y[i] = earth_y[i]
        sc_z[i] = earth_z[i]
        sc_state[i] = "On Earth Orbit"


# --- Plotting and Animation ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Plot orbits
ax.plot(earth_x, earth_y, earth_z, linestyle=":", color="blue", label="Earth Orbit")
ax.plot(mars_x, mars_y, mars_z, linestyle=":", color="red", label="Mars Orbit")

# Sun
ax.scatter([0], [0], [0], color="yellow", s=200, label="Sun")

# Initial positions - these lines will be updated
(earth_line,) = ax.plot([], [], [], "o", color="blue", markersize=7, label="Earth")
(mars_line,) = ax.plot([], [], [], "o", color="red", markersize=5, label="Mars")
(sc_line,) = ax.plot([], [], [], "o", color="lime", markersize=4, label="Spacecraft")
(sc_traj,) = ax.plot(
    [], [], [], "-", color="lime", linewidth=1, label="Trajectory"
)  # Spacecraft trajectory trace

# Time and State Text
time_text = ax.text2D(
    0.05,
    0.95,
    "",
    transform=ax.transAxes,
    color="white",
    bbox=dict(facecolor="black", alpha=0.5),
)
state_text = ax.text2D(
    0.05,
    0.90,
    "",
    transform=ax.transAxes,
    color="white",
    bbox=dict(facecolor="black", alpha=0.5),
)


# Set plot limits and labels
max_range = MARS_RADIUS_ORBIT * 1.2
ax.set_xlim([-max_range, max_range])
ax.set_ylim([-max_range, max_range])
ax.set_zlim([-max_range, max_range])  # Give some z-depth even if orbits are coplanar
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth-Mars Round Trip Simulation")
ax.legend(loc="center left", bbox_to_anchor=(1.05, 0.5))
ax.set_aspect("auto")  # 'equal' might distort view if z-limits differ significantly
# Optional: Set background color
ax.set_facecolor("black")
fig.patch.set_facecolor("darkgrey")


# Animation function
sc_traj_x, sc_traj_y, sc_traj_z = [], [], []  # Store trajectory points for trace


def update(frame):
    i = frame

    # Update planet positions
    earth_line.set_data_3d([earth_x[i]], [earth_y[i]], [earth_z[i]])
    mars_line.set_data_3d([mars_x[i]], [mars_y[i]], [mars_z[i]])

    # Update spacecraft position
    sc_line.set_data_3d([sc_x[i]], [sc_y[i]], [sc_z[i]])

    # Update spacecraft trajectory trace
    # Only plot trace when relevant (e.g., during transfer or full mission)
    if i >= launch_idx_1:
        if i == launch_idx_1:  # Reset trace at first launch
            sc_traj_x.clear()
            sc_traj_y.clear()
            sc_traj_z.clear()
        sc_traj_x.append(sc_x[i])
        sc_traj_y.append(sc_y[i])
        sc_traj_z.append(sc_z[i])
        sc_traj.set_data_3d(sc_traj_x, sc_traj_y, sc_traj_z)

    # Update text
    time_text.set_text(f"Time: {times[i]:.1f} days ({times[i]/365.25:.2f} years)")
    state_text.set_text(f"State: {sc_state[i]}")

    # Return updated plot elements
    return earth_line, mars_line, sc_line, sc_traj, time_text, state_text


# Create animation
ani = animation.FuncAnimation(
    fig, update, frames=N_STEPS, interval=20, blit=True, repeat=False
)  # interval in ms

# To save the animation (requires ffmpeg or imagemagick):
# print("\nSaving animation... This might take a while.")
# ani.save('earth_mars_round_trip.mp4', writer='ffmpeg', fps=30, dpi=150)
# print("Animation saved as earth_mars_round_trip.mp4")

plt.tight_layout(rect=[0, 0, 0.9, 1])  # Adjust layout to make space for legend
plt.show()
