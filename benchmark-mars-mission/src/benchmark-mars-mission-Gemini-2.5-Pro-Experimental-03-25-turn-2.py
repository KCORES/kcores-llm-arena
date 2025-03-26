import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

# --- Configuration & Constants ---

# Astronomical Unit (km) - for scale reference if needed, but we'll use AU directly
AU = 1.496e8

# Orbital Parameters (Simplified - Circular Orbits)
EARTH_ORBIT_RADIUS = 1.0  # AU
MARS_ORBIT_RADIUS = 1.524  # AU
EARTH_ORBITAL_PERIOD = 365.25  # Earth days
MARS_ORBITAL_PERIOD = 687.0  # Earth days

# Mission Timing (Approximate - for visualization)
# Mars needs to be ahead of Earth for Hohmann transfer
MARS_LEAD_ANGLE_DEG = 44.0
# Calculate initial phase offset for Mars based on lead angle
MARS_INITIAL_PHASE = np.deg2rad(MARS_LEAD_ANGLE_DEG)
MARS_TIME_OFFSET = (MARS_INITIAL_PHASE / (2 * np.pi)) * MARS_ORBITAL_PERIOD  # Days

TRAVEL_TIME_TO_MARS = 260  # Days (approximate one-way Hohmann)
# Wait time on Mars until next Earth return window (approx. Synodic period - travel times)
# Synodic period (Earth-Mars) ~ 780 days
WAIT_ON_MARS = 490  # Days (approximate, ensures reasonable alignment for return)
TRAVEL_TIME_TO_EARTH = 260  # Days (approximate)

# Simulation Time
T0_LAUNCH_EARTH = 0.0
T1_ARRIVE_MARS = T0_LAUNCH_EARTH + TRAVEL_TIME_TO_MARS
T2_LAUNCH_MARS = T1_ARRIVE_MARS + WAIT_ON_MARS
T3_ARRIVE_EARTH = T2_LAUNCH_MARS + TRAVEL_TIME_TO_EARTH

TOTAL_SIMULATION_TIME = T3_ARRIVE_EARTH + 60  # Add some buffer time after arrival
TIME_STEP = 5  # Days per frame
N_FRAMES = int(TOTAL_SIMULATION_TIME / TIME_STEP)

# Animation settings
INTERVAL_MS = 30  # Milliseconds between frames (controls speed)
SAVE_ANIMATION = False  # Set to True to save as GIF (requires imagemagick)
GIF_FILENAME = "earth_mars_round_trip.gif"

# --- Helper Functions ---


def get_planet_position(radius, period, time, phase_offset_time=0.0):
    """Calculates planet position on a circular orbit at a given time."""
    angle = 2 * np.pi * (time + phase_offset_time) / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0  # Coplanar orbit
    return x, y, z


def interpolate_trajectory(start_pos, end_pos, fraction):
    """Simple linear interpolation for trajectory visualization (not physically accurate)."""
    # A slightly better visual: interpolate radius and angle separately for a spiral effect
    start_radius = np.sqrt(start_pos[0] ** 2 + start_pos[1] ** 2)
    end_radius = np.sqrt(end_pos[0] ** 2 + end_pos[1] ** 2)
    current_radius = start_radius + (end_radius - start_radius) * fraction

    start_angle = np.arctan2(start_pos[1], start_pos[0])
    end_angle = np.arctan2(end_pos[1], end_pos[0])

    # Handle angle wrapping properly (shortest path)
    delta_angle = end_angle - start_angle
    if delta_angle > np.pi:
        delta_angle -= 2 * np.pi
    elif delta_angle < -np.pi:
        delta_angle += 2 * np.pi

    current_angle = start_angle + delta_angle * fraction

    x = current_radius * np.cos(current_angle)
    y = current_radius * np.sin(current_angle)
    z = (
        start_pos[2] + (end_pos[2] - start_pos[2]) * fraction
    )  # Assume Z stays 0 for coplanar

    return x, y, z


# --- Set up the Plot ---
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("black")  # Space background
fig.patch.set_facecolor("black")  # Figure background outside axes

# Plot Style
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False
ax.xaxis.pane.set_edgecolor("k")
ax.yaxis.pane.set_edgecolor("k")
ax.zaxis.pane.set_edgecolor("k")
ax.grid(False)  # Hide grid lines
# Use white ticks and labels
ax.tick_params(axis="x", colors="white")
ax.tick_params(axis="y", colors="white")
ax.tick_params(axis="z", colors="white")
ax.xaxis.label.set_color("white")
ax.yaxis.label.set_color("white")
ax.zaxis.label.set_color("white")


# Plot Sun
ax.scatter([0], [0], [0], color="yellow", s=200, label="Sun")

# Plot Orbits (Static)
orbit_angles = np.linspace(0, 2 * np.pi, 200)
earth_orbit_x = EARTH_ORBIT_RADIUS * np.cos(orbit_angles)
earth_orbit_y = EARTH_ORBIT_RADIUS * np.sin(orbit_angles)
ax.plot(
    earth_orbit_x,
    earth_orbit_y,
    [0] * len(orbit_angles),
    color="dodgerblue",
    linestyle="--",
    linewidth=0.8,
    label="Earth Orbit",
)

mars_orbit_x = MARS_ORBIT_RADIUS * np.cos(orbit_angles)
mars_orbit_y = MARS_ORBIT_RADIUS * np.sin(orbit_angles)
ax.plot(
    mars_orbit_x,
    mars_orbit_y,
    [0] * len(orbit_angles),
    color="red",
    linestyle="--",
    linewidth=0.8,
    label="Mars Orbit",
)

# Initialize Dynamic Plot Elements (Planets and Spacecraft)
(earth_plot,) = ax.plot([], [], [], "o", color="blue", markersize=8, label="Earth")
(mars_plot,) = ax.plot([], [], [], "o", color="orangered", markersize=6, label="Mars")
(sc_plot,) = ax.plot([], [], [], "o", color="lime", markersize=4, label="Spacecraft")
(sc_trail,) = ax.plot(
    [], [], [], "-", color="lime", linewidth=0.5, alpha=0.7
)  # Spacecraft trail

# Time display
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, color="white")

# Store trail points
sc_trail_x, sc_trail_y, sc_trail_z = [], [], []


# --- Animation Function ---
def update(frame):
    current_time = frame * TIME_STEP

    # --- Update Planet Positions ---
    earth_pos = get_planet_position(
        EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD, current_time
    )
    mars_pos = get_planet_position(
        MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD, current_time, MARS_TIME_OFFSET
    )

    earth_plot.set_data_3d([earth_pos[0]], [earth_pos[1]], [earth_pos[2]])
    mars_plot.set_data_3d([mars_pos[0]], [mars_pos[1]], [mars_pos[2]])

    # --- Update Spacecraft Position ---
    sc_pos = (0, 0, 0)  # Default
    current_phase = ""

    if current_time < T0_LAUNCH_EARTH:
        # Before launch - stays with Earth (optional visualization)
        # sc_pos = earth_pos
        # For simplicity, start trajectory from T0
        sc_pos = get_planet_position(
            EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD, T0_LAUNCH_EARTH
        )
        current_phase = "On Earth (Pre-Launch)"

    elif T0_LAUNCH_EARTH <= current_time < T1_ARRIVE_MARS:
        # Phase 1: Earth to Mars
        fraction = (current_time - T0_LAUNCH_EARTH) / TRAVEL_TIME_TO_MARS
        start_pos = get_planet_position(
            EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD, T0_LAUNCH_EARTH
        )
        # Target the position Mars WILL be at upon arrival
        end_pos = get_planet_position(
            MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD, T1_ARRIVE_MARS, MARS_TIME_OFFSET
        )
        sc_pos = interpolate_trajectory(start_pos, end_pos, fraction)
        current_phase = "Transfer: Earth -> Mars"

    elif T1_ARRIVE_MARS <= current_time < T2_LAUNCH_MARS:
        # Phase 2: On Mars Surface (or low orbit) - follow Mars
        sc_pos = mars_pos
        current_phase = (
            f"On Mars (Waiting... {int(T2_LAUNCH_MARS - current_time)} days left)"
        )

    elif T2_LAUNCH_MARS <= current_time < T3_ARRIVE_EARTH:
        # Phase 3: Mars to Earth
        fraction = (current_time - T2_LAUNCH_MARS) / TRAVEL_TIME_TO_EARTH
        # Depart from Mars' position at T2
        start_pos = get_planet_position(
            MARS_ORBIT_RADIUS, MARS_ORBITAL_PERIOD, T2_LAUNCH_MARS, MARS_TIME_OFFSET
        )
        # Target the position Earth WILL be at upon arrival
        end_pos = get_planet_position(
            EARTH_ORBIT_RADIUS, EARTH_ORBITAL_PERIOD, T3_ARRIVE_EARTH
        )
        sc_pos = interpolate_trajectory(start_pos, end_pos, fraction)
        current_phase = "Transfer: Mars -> Earth"

    else:  # current_time >= T3_ARRIVE_EARTH
        # Phase 4: Arrived back at Earth - follow Earth
        sc_pos = earth_pos
        current_phase = "Back on Earth"

    sc_plot.set_data_3d([sc_pos[0]], [sc_pos[1]], [sc_pos[2]])

    # Update trail (only during transfer phases)
    if (T0_LAUNCH_EARTH <= current_time < T1_ARRIVE_MARS) or (
        T2_LAUNCH_MARS <= current_time < T3_ARRIVE_EARTH
    ):
        sc_trail_x.append(sc_pos[0])
        sc_trail_y.append(sc_pos[1])
        sc_trail_z.append(sc_pos[2])
        sc_trail.set_data_3d(sc_trail_x, sc_trail_y, sc_trail_z)
    elif current_time >= T3_ARRIVE_EARTH:  # Optional: clear trail on arrival
        pass  # Keep trail for now

    # Update time text
    time_text.set_text(
        f"Day: {int(current_time)} / {int(TOTAL_SIMULATION_TIME)}\nPhase: {current_phase}"
    )

    # Return list of modified artists for blitting
    return earth_plot, mars_plot, sc_plot, sc_trail, time_text


# --- Set Axes Limits and Aspect Ratio ---
max_radius = MARS_ORBIT_RADIUS * 1.1
ax.set_xlim(-max_radius, max_radius)
ax.set_ylim(-max_radius, max_radius)
ax.set_zlim(-max_radius, max_radius)
ax.set_aspect("equal", adjustable="box")  # Crucial for circular orbits to look circular

# Set labels
ax.set_xlabel("X (AU)", color="white")
ax.set_ylabel("Y (AU)", color="white")
ax.set_zlabel("Z (AU)", color="white")
ax.set_title(
    "Simplified Earth-Mars Round Trip Animation", color="white", y=1.05
)  # Adjust title position

# Add Legend
legend = ax.legend(
    facecolor="dimgray",
    edgecolor="white",
    labelcolor="white",
    loc="upper right",
    bbox_to_anchor=(1.15, 0.9),
)
for text in legend.get_texts():
    text.set_color("white")


# --- Create and Run Animation ---
# Blit=True generally gives smoother animation, but can be tricky. If issues arise, try blit=False.
ani = animation.FuncAnimation(
    fig, update, frames=N_FRAMES, interval=INTERVAL_MS, blit=True, repeat=False
)

# --- Save or Show ---
if SAVE_ANIMATION:
    print(f"Saving animation to {GIF_FILENAME}... (this may take a while)")
    try:
        # You might need to specify the full path to magick if it's not in your system PATH
        # plt.rcParams['animation.convert_path'] = r'C:\Program Files\ImageMagick-7.1.1-Q16-HDRI\magick.exe' # Example for Windows
        ani.save(GIF_FILENAME, writer="imagemagick", fps=1000 / INTERVAL_MS)
        print("Animation saved successfully.")
    except Exception as e:
        print(f"Error saving animation: {e}")
        print("Ensure ImageMagick is installed and accessible in your system's PATH.")
        print("Showing plot instead...")
        plt.show()
else:
    plt.show()

print("Animation finished.")
