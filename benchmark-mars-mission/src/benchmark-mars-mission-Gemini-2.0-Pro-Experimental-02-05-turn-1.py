import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.constants import G  # Gravitational constant
from datetime import datetime, timedelta

# --- Constants and Parameters ---

#  Orbital Parameters (Simplified for visualization - these are NOT accurate orbital mechanics)
#  Using Keplerian elements (semi-major axis, eccentricity) *very roughly* approximated, and
#  making large simplifying assumptions (e.g., circular orbits, ignoring inclinations).
#  Realistic trajectory calculations require much more complex numerical integration.
earth_orbital_radius = (
    1.0  # AU (Astronomical Units - arbitrary units for visualization)
)
mars_orbital_radius = 1.52  # AU
earth_orbital_period = 365.25  # days
mars_orbital_period = 687.0  # days
sun_mass = 1.0  # Arbitrary units, for scaling gravitational force visualization

# Launch and Transfer Parameters
launch_date = datetime(2024, 1, 1)  # Example launch date
transfer_duration = (
    250  # Days (approximate Hohmann transfer duration - very simplified!)
)
mars_stay_duration = 550  # Days (approx. waiting for the next launch window)
return_transfer_duration = 250  # Days

# Animation parameters
animation_duration = 60  # seconds
frames_per_second = 30
total_frames = animation_duration * frames_per_second
time_step = (
    earth_orbital_period * 4
) / total_frames  #  Simulate several years, adjust for visual clarity.

# --- Helper Functions ---


def orbital_position(radius, period, time):
    """Calculates the position of a planet in its orbit.

    Args:
        radius: The orbital radius (semi-major axis).
        period: The orbital period in days.
        time: The time in days since a reference point (e.g., launch date).

    Returns:
        A tuple (x, y) representing the planet's position in the orbital plane.
    """
    angle = 2 * np.pi * (time / period)
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    return x, y


def hohmann_transfer(r1, r2, t, total_time):
    """Simulates a simplified Hohmann transfer trajectory.

    Args:
      r1: Initial orbital radius
      r2: Final orbital radius
      t: current time during transfer
      total_time: total transfer time

    Returns:
        x, y position during transfer.
    """
    # VERY simplified Hohmann transfer.  Assumes instantaneous velocity changes
    # and ignores many real-world factors.  Just for visualization.
    transfer_angle = np.pi * (t / total_time)  # Angle covered during transfer
    current_radius = r1 + (r2 - r1) * (
        t / total_time
    )  # Linearly interpolate the radius (NOT ACCURATE)
    x = current_radius * np.cos(transfer_angle)
    y = current_radius * np.sin(transfer_angle)
    return x, y


# ---  Setup the 3D Plot ---

fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")

# Plot the Sun
ax.scatter(0, 0, 0, color="yellow", s=100, label="Sun")

# Initial positions of Earth and Mars
earth_x, earth_y = orbital_position(earth_orbital_radius, earth_orbital_period, 0)
mars_x, mars_y = orbital_position(mars_orbital_radius, mars_orbital_period, 0)

# Create plot objects for Earth, Mars, and the spacecraft.  These will be updated in the animation.
(earth,) = ax.plot(
    [earth_x], [earth_y], [0], marker="o", color="blue", markersize=8, label="Earth"
)
(mars,) = ax.plot(
    [mars_x], [mars_y], [0], marker="o", color="red", markersize=6, label="Mars"
)
(spacecraft,) = ax.plot(
    [], [], [], marker="*", color="cyan", markersize=4, label="Spacecraft"
)

# Trajectory lines (will be updated dynamically during animation)
(trajectory_line,) = ax.plot([], [], [], color="cyan", linestyle="--", alpha=0.7)

# --- Animation Function ---


def animate(frame):
    current_time = frame * time_step
    current_date = launch_date + timedelta(days=current_time)

    # Calculate positions of Earth and Mars
    earth_x, earth_y = orbital_position(
        earth_orbital_radius, earth_orbital_period, current_time
    )
    mars_x, mars_y = orbital_position(
        mars_orbital_radius, mars_orbital_period, current_time
    )

    # --- Spacecraft Trajectory Logic ---

    if current_time < transfer_duration:  # Outbound Hohmann transfer
        sc_x, sc_y = hohmann_transfer(
            earth_orbital_radius, mars_orbital_radius, current_time, transfer_duration
        )
        sc_z = 0  # Keep the spacecraft in the orbital plane (for simplicity)
        trajectory_line.set_data_3d(
            np.append(trajectory_line._verts3d[0], sc_x),
            np.append(trajectory_line._verts3d[1], sc_y),
            np.append(trajectory_line._verts3d[2], sc_z),
        )

    elif current_time < transfer_duration + mars_stay_duration:  # Stay on Mars
        sc_x, sc_y = mars_x, mars_y
        sc_z = 0
        # No need to update trajectory_line during the stay on Mars.

    elif (
        current_time < transfer_duration + mars_stay_duration + return_transfer_duration
    ):  # Return Hohmann Transfer
        time_since_mars_departure = current_time - (
            transfer_duration + mars_stay_duration
        )
        sc_x, sc_y = hohmann_transfer(
            mars_orbital_radius,
            earth_orbital_radius,
            time_since_mars_departure,
            return_transfer_duration,
        )
        sc_z = 0
        trajectory_line.set_data_3d(
            np.append(trajectory_line._verts3d[0], sc_x),
            np.append(trajectory_line._verts3d[1], sc_y),
            np.append(trajectory_line._verts3d[2], sc_z),
        )

    else:  #  Mission Complete - spacecraft has returned
        # Could add logic here to keep the spacecraft with Earth after return,
        # or stop the animation.  For now, let it keep running.
        sc_x, sc_y = earth_x, earth_y  # Simplification: stay near Earth.
        sc_z = 0

    # Update plot objects
    earth.set_data_3d([earth_x], [earth_y], [0])
    mars.set_data_3d([mars_x], [mars_y], [0])
    spacecraft.set_data_3d([sc_x], [sc_y], [sc_z])

    # ---  Labels and Display ---
    ax.set_title(f"Earth-Mars-Earth Mission\nDate: {current_date.strftime('%Y-%m-%d')}")
    return earth, mars, spacecraft, trajectory_line


# ---  Set Plot Limits and Labels ---
max_range = max(earth_orbital_radius, mars_orbital_radius) * 1.2  # Add some padding
ax.set_xlim(-max_range, max_range)
ax.set_ylim(-max_range, max_range)
ax.set_zlim(
    -max_range / 4, max_range / 4
)  #  Keep z-axis smaller for better perspective
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.legend()

# --- Create and Run the Animation ---

ani = FuncAnimation(
    fig, animate, frames=total_frames, interval=1000 / frames_per_second, blit=True
)  # interval in milliseconds

# Save the animation (optional) - requires ffmpeg or similar installed
# ani.save("earth_mars_mission.mp4", writer='ffmpeg', fps=frames_per_second)

plt.show()
