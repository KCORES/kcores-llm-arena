import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# -----------------------------
# Constants and Parameters
# -----------------------------

# Orbital Radii in AU
earth_r = 1.0
mars_r = 1.524

# Orbital Periods in Years
earth_T = 1.0
mars_T = 1.88

# Angular velocities in radians/year
omega_e = 2 * np.pi / earth_T
omega_m = 2 * np.pi / mars_T

# Mission Phases Duration in Years
tau_outbound = 0.7
tau_dwell = 1.5
tau_return = tau_outbound
total_time = tau_outbound + tau_dwell + tau_return

# Initial angle of Mars at t=0 (arbitrary for visual clarity)
theta_m_initial = -np.pi / 2

# Sun radius for visualization
sun_r = 0.1

# Number of frames
num_frames = 200


# -----------------------------
# Position Functions
# -----------------------------


def earth_position(t):
    """Returns Earth's position at time t"""
    angle = omega_e * t
    x = earth_r * np.cos(angle)
    y = earth_r * np.sin(angle)
    return x, y, 0


def mars_position(t):
    """Returns Mars's position at time t"""
    angle = theta_m_initial + omega_m * t
    x = mars_r * np.cos(angle)
    y = mars_r * np.sin(angle)
    return x, y, 0


def hohmann_ellipse(s, r1, r2):
    """
    Returns (x, y) position on Hohmann transfer ellipse for progress s ∈ [0,1]
    Approximation: True anomaly ν = π*s
    """
    a = (r1 + r2) / 2
    e = (r2 - r1) / (r1 + r2)
    nu = np.pi * s
    r = a * (1 - e**2) / (1 + e * np.cos(nu))
    x = r * np.cos(nu)
    y = r * np.sin(nu)
    return x, y


def ship_position(t):
    """
    Returns spacecraft position at time t
    Based on mission phases:
        - Outbound (to Mars)
        - Dwell (on Mars)
        - Return (to Earth)
    """
    if t < tau_outbound:
        # Phase 1: Earth to Mars (Hohmann)
        s = t / tau_outbound
        x, y = hohmann_ellipse(s, earth_r, mars_r)
        return x, y, 0

    elif t < tau_outbound + tau_dwell:
        # Phase 2: On Mars
        return mars_position(t)

    else:
        # Phase 3: Mars to Earth (Hohmann)
        s = (t - (tau_outbound + tau_dwell)) / tau_return
        x, y = hohmann_ellipse(s, mars_r, earth_r)
        # Rotate to match Mars' orientation at departure
        angle = theta_m_initial + omega_m * (tau_outbound + tau_dwell)
        rot_x = x * np.cos(angle) - y * np.sin(angle)
        rot_y = x * np.sin(angle) + y * np.cos(angle)
        return rot_x, rot_y, 0


# -----------------------------
# Setup Plotting Environment
# -----------------------------

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Orbit Grids
theta = np.linspace(0, 2 * np.pi, 360)
earth_orbit_x = earth_r * np.cos(theta)
earth_orbit_y = earth_r * np.sin(theta)
mars_orbit_x = mars_r * np.cos(theta)
mars_orbit_y = mars_r * np.sin(theta)

ax.plot(earth_orbit_x, earth_orbit_y, np.zeros_like(theta), "b--", lw=1, alpha=0.5)
ax.plot(mars_orbit_x, mars_orbit_y, np.zeros_like(theta), "r--", lw=1, alpha=0.5)

# Sun
u = np.linspace(0, 2 * np.pi, 50)
v = np.linspace(0, np.pi, 25)
x_sun = sun_r * np.outer(np.cos(u), np.sin(v))
y_sun = sun_r * np.outer(np.sin(u), np.sin(v))
z_sun = sun_r * np.outer(np.ones_like(u), np.cos(v))
ax.plot_surface(x_sun, y_sun, z_sun, color="gold", alpha=0.9)

# Initialization of dots
(earth_dot,) = ax.plot([], [], [], "bo", ms=6, label="Earth")
(mars_dot,) = ax.plot([], [], [], "ro", ms=6, label="Mars")
(ship_dot,) = ax.plot([], [], [], "go", ms=4, label="Spacecraft")
(ship_line,) = ax.plot([], [], [], "g-", lw=1, alpha=0.6)

# Axis Settings
lim = 1.7 * mars_r
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-0.2, 0.2)
ax.set_title("Earth to Mars Mission Simulation")
ax.legend(loc="upper right")
ax.grid(False)
ax.axis("off")


# -----------------------------
# Animation Update Function
# -----------------------------


def update(frame):
    t = frame * total_time / num_frames

    # Update positions
    ex, ey, ez = earth_position(t)
    mx, my, mz = mars_position(t)
    sx, sy, sz = ship_position(t)

    # Update dots
    earth_dot.set_data([ex], [ey])
    earth_dot.set_3d_properties([ez])

    mars_dot.set_data([mx], [my])
    mars_dot.set_3d_properties([mz])

    ship_dot.set_data([sx], [sy])
    ship_dot.set_3d_properties([sz])

    # Update trajectory line
    time_steps = np.linspace(0, t, max(2, int(50 * t)))
    xs, ys, zs = [], [], []
    for ti in time_steps:
        xi, yi, zi = ship_position(ti)
        xs.append(xi)
        ys.append(yi)
        zs.append(zi)
    ship_line.set_data(xs, ys)
    ship_line.set_3d_properties(zs)

    return earth_dot, mars_dot, ship_dot, ship_line


# -----------------------------
# Run Animation
# -----------------------------

ani = FuncAnimation(fig, update, frames=num_frames, interval=50, repeat=True, blit=True)

# Save animation (optional)
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', dpi=150)

plt.show()
