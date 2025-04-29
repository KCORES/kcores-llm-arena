import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ----------------------------
# Constants and Configuration
# ----------------------------

# Distance units in Astronomical Units (AU)
AU = 1.0
EARTH_ORBIT_RADIUS = 1.0 * AU
MARS_ORBIT_RADIUS = 1.524 * AU

# Orbital periods in years
EARTH_YEAR = 1.0
MARS_YEAR = 1.882

# Angular velocities in radians/year
omega_earth = 2 * np.pi / EARTH_YEAR
omega_mars = 2 * np.pi / MARS_YEAR

# Estimated Hohmann transfer time (Earth to Mars) in years
transit_time = 0.71  # Approximation based on real-world estimates

# Synodic period for Earth-Mars launch window
synodic_period = 1.0 / (1.0 / EARTH_YEAR - 1.0 / MARS_YEAR)
wait_time_on_mars = synodic_period - transit_time  # Optimal wait before return

# Total mission time
total_time = transit_time + wait_time_on_mars + transit_time

# Number of frames for animation
num_frames = 500
t_anim = np.linspace(0, total_time, num_frames)

# Control offset for Bezier curves (visual effect)
bezier_offset = 1.5  # AU

# ----------------------------
# Helper Functions
# ----------------------------


def earth_position(t):
    """Returns Earth's position at time t."""
    x = EARTH_ORBIT_RADIUS * np.cos(omega_earth * t)
    y = EARTH_ORBIT_RADIUS * np.sin(omega_earth * t)
    return x, y


def mars_position(t):
    """Returns Mars' position at time t."""
    x = MARS_ORBIT_RADIUS * np.cos(omega_mars * t + np.pi)  # Start at (−r, 0)
    y = MARS_ORBIT_RADIUS * np.sin(omega_mars * t + np.pi)
    return x, y


def sc_position(t_current):
    """
    Returns spacecraft's position depending on the mission phase.
    Phases:
      1. Earth to Mars transfer
      2. Waiting on Mars
      3. Mars to Earth transfer
    """
    if t_current <= transit_time:
        # Phase 1: Earth to Mars transfer
        tau = t_current / transit_time
        xe0, ye0 = earth_position(0.0)
        xm1, ym1 = mars_position(transit_time)
        # Use a Bezier curve with a top control point
        xc, yc = 0.0, bezier_offset
        x = (1 - tau) ** 2 * xe0 + 2 * (1 - tau) * tau * xc + tau**2 * xm1
        y = (1 - tau) ** 2 * ye0 + 2 * (1 - tau) * tau * yc + tau**2 * ym1
        return x, y

    elif t_current <= transit_time + wait_time_on_mars:
        # Phase 2: On Mars
        return mars_position(t_current)

    else:
        # Phase 3: Mars to Earth transfer
        t_rel = t_current - (transit_time + wait_time_on_mars)
        tau = t_rel / transit_time
        xm0, ym0 = mars_position(transit_time + wait_time_on_mars)
        xe1, ye1 = earth_position(transit_time + wait_time_on_mars + transit_time)
        # Use a bottom control point for return arc
        xc, yc = 0.0, -bezier_offset
        x = (1 - tau) ** 2 * xm0 + 2 * (1 - tau) * tau * xc + tau**2 * xe1
        y = (1 - tau) ** 2 * ym0 + 2 * (1 - tau) * tau * yc + tau**2 * ye1
        return x, y


# ----------------------------
# Set Up the Plot
# ----------------------------

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Limit Z-axis to keep it mostly planar
ax.set_zlim(-0.5, 0.5)

# Plot the Sun
(sun,) = ax.plot([0], [0], [0], "yo", markersize=10, label="Sun")

# Plot Earth's orbit
theta = np.linspace(0, 2 * np.pi, 360)
xe_orbit = EARTH_ORBIT_RADIUS * np.cos(theta)
ye_orbit = EARTH_ORBIT_RADIUS * np.sin(theta)
ax.plot(xe_orbit, ye_orbit, np.zeros_like(theta), "c--", lw=1, label="Earth's Orbit")

# Plot Mars' orbit
xm_orbit = MARS_ORBIT_RADIUS * np.cos(theta)
ym_orbit = MARS_ORBIT_RADIUS * np.sin(theta)
ax.plot(xm_orbit, ym_orbit, np.zeros_like(theta), "r--", lw=1, label="Mars' Orbit")

# Initialize objects
(earth,) = ax.plot([], [], [], "go", markersize=8, label="Earth")
(mars,) = ax.plot([], [], [], "ro", markersize=6, label="Mars")
(spacecraft,) = ax.plot([], [], [], "k^", markersize=6, label="Spacecraft")

ax.legend(loc="upper left")
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth to Mars Mission Animation")
ax.view_init(elev=10, azim=-45)

# ----------------------------
# Animation Function
# ----------------------------


def update(frame):
    t_current = t_anim[frame]

    # Update Earth
    xe, ye = earth_position(t_current)
    earth.set_data([xe], [ye])
    earth.set_3d_properties([0])

    # Update Mars
    xm, ym = mars_position(t_current)
    mars.set_data([xm], [ym])
    mars.set_3d_properties([0])

    # Update Spacecraft
    xs, ys = sc_position(t_current)
    spacecraft.set_data([xs], [ys])
    spacecraft.set_3d_properties([0])

    # Update Title with Current Time
    ax.set_title(f"Earth to Mars Mission | Time: {t_current:.2f} years")

    return earth, mars, spacecraft


# ----------------------------
# Run Animation
# ----------------------------

ani = FuncAnimation(fig, update, frames=num_frames, interval=20, blit=True)

# Optional: Save to file
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30)

plt.show()
