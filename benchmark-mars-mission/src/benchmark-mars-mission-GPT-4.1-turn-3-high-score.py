import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# PARAMETERS (arbitrary units, all circular & coplanar)
R_earth = 1.0  # Earth orbit radius (AU)
R_mars = 1.52  # Mars orbit radius (AU)
T_earth = 1.0  # Earth orbital period (year)
T_mars = 1.88  # Mars orbital period (year)

# Transfer times (Hohmann transfer details)
from math import pi, sqrt

G = 1  # Gravitational const, arbitrary units
M = 1  # Sun mass
a_transfer = (R_earth + R_mars) / 2  # semi-major axis of transfer
T_transfer = pi * sqrt(a_transfer**3 / (G * M))  # half period for Hohmann in years

# Launch Windows
synodic_period = 1 / abs(1 / T_earth - 1 / T_mars)  # time between opportunities

# Animation parameters
frames_per_year = 100
total_time = T_transfer * 2 + synodic_period * 0.2  # Slight "wait" time at Mars
dt = 1.0 / frames_per_year  # in years
n_frames = int(total_time / dt)


# Create orbital locations over time
def orbit_pos(radius, period, t):
    angle = 2 * np.pi * t / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    return np.stack((x, y), axis=-1)


# Build up arrays for Earth, Mars over animation
t_arr = np.linspace(0, total_time, n_frames)
earth_pos = orbit_pos(R_earth, T_earth, t_arr)
mars_pos = orbit_pos(R_mars, T_mars, t_arr)


# Spacecraft state machine
def spacecraft_pos(t):
    # Launch from Earth at t=0
    # Transfer to Mars over T_transfer
    # Wait at Mars until next window (make it simple, stay for half synodic period)
    # Transfer back to Earth over T_transfer
    t1 = T_transfer
    t_wait = synodic_period / 2
    t2 = t1 + t_wait
    t3 = t2 + T_transfer

    if t < t1:  # Outbound
        frac = t / T_transfer
        theta_start = 2 * np.pi * 0 / T_earth  # Starting at t=0
        theta_end = 2 * np.pi * T_transfer / T_mars  # where Mars will be at arrival
        angle = theta_start + (theta_end - theta_start) * frac
        radius = R_earth * (1 - frac) + R_mars * frac
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        return x, y
    elif t < t2:  # On Mars
        # Stick at Mars's position
        idx = np.argmin(np.abs(t_arr - t))
        return mars_pos[idx]
    elif t < t3:  # Return
        t_since = t - t2
        frac = t_since / T_transfer
        theta_start = 2 * np.pi * t2 / T_mars  # Starting from Mars
        theta_end = 2 * np.pi * (t2 + T_transfer) / T_earth
        angle = theta_start + (theta_end - theta_start) * frac
        radius = R_mars * (1 - frac) + R_earth * frac
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        return x, y
    else:
        # Arrived back at Earth, stick at Earth
        idx = np.argmin(np.abs(t_arr - t))
        return earth_pos[idx]


# --- PLOTTING SETUP ---
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.05, 0.05)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z")

# Sun
ax.scatter([0], [0], [0], color="yellow", s=200, label="Sun", zorder=10)

# Orbits
theta_line = np.linspace(0, 2 * np.pi, 200)
ax.plot(R_earth * np.cos(theta_line), R_earth * np.sin(theta_line), 0, "b--", alpha=0.5)
ax.plot(R_mars * np.cos(theta_line), R_mars * np.sin(theta_line), 0, "r--", alpha=0.45)

# IMAGES
earth_scat = ax.scatter([], [], [], color="b", s=100, label="Earth", zorder=10)
mars_scat = ax.scatter([], [], [], color="red", s=120, label="Mars", zorder=10)
sc_scat = ax.scatter([], [], [], color="magenta", s=40, label="Spacecraft", zorder=12)

# ORBIT TRAILS
trail_len = int(0.2 * frames_per_year)  # show last N frames as trail
(earth_trail,) = ax.plot([], [], [], color="b", lw=1.7)
(mars_trail,) = ax.plot([], [], [], color="r", lw=2)
(sc_trail,) = ax.plot([], [], [], color="magenta", lw=2)

# TIME LABEL
time_text = ax.text2D(0.04, 0.95, "", transform=ax.transAxes, fontsize=14)

ax.legend(loc="lower right")


# Update function for animation
def update(i):
    # Earth and Mars
    earth_xy = earth_pos[i]
    mars_xy = mars_pos[i]
    earth_scat._offsets3d = ([earth_xy[0]], [earth_xy[1]], [0])
    mars_scat._offsets3d = ([mars_xy[0]], [mars_xy[1]], [0])

    # Trails
    istart = max(0, i - trail_len)
    # Leave Z=0 for planetary positions
    earth_trail.set_data(earth_pos[istart : i + 1, 0], earth_pos[istart : i + 1, 1])
    earth_trail.set_3d_properties([0] * (i + 1 - istart))
    mars_trail.set_data(mars_pos[istart : i + 1, 0], mars_pos[istart : i + 1, 1])
    mars_trail.set_3d_properties([0] * (i + 1 - istart))

    # Spacecraft
    t_now = t_arr[i]
    sc_x, sc_y = spacecraft_pos(t_now)
    sc_scat._offsets3d = ([sc_x], [sc_y], [0.01])
    # Spacecraft trail
    sc_xs, sc_ys = [], []
    for j in range(max(0, i - trail_len), i + 1):
        x, y = spacecraft_pos(t_arr[j])
        sc_xs.append(x)
        sc_ys.append(y)
    sc_trail.set_data(sc_xs, sc_ys)
    sc_trail.set_3d_properties([0.01] * len(sc_xs))

    # Update time label
    time_text.set_text(f"Time: {t_now:.2f} years")
    return earth_scat, mars_scat, sc_scat, earth_trail, mars_trail, sc_trail, time_text


ani = FuncAnimation(fig, update, frames=n_frames, interval=30, blit=False)
plt.title("Animated Earth↔Mars Mission in 3D")
plt.show()
