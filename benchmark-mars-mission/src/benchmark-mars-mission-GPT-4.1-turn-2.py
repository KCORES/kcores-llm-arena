import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Orbital radii (AU)
r_earth = 1.0
r_mars = 1.524

# Orbital periods (years)
T_earth = 1.0
T_mars = 1.881

n_frames = 400
total_time = 4  # Total simulation time in years
t = np.linspace(0, total_time, n_frames)

w_earth = 2 * np.pi / T_earth
w_mars = 2 * np.pi / T_mars

# Calculate Hohmann transfer time
a_transfer = (r_earth + r_mars) / 2
T_transfer = np.pi * np.sqrt(a_transfer**3)

earth_pos = np.column_stack(
    (r_earth * np.cos(w_earth * t), r_earth * np.sin(w_earth * t), np.zeros_like(t))
)
mars_pos = np.column_stack(
    (
        r_mars * np.cos(w_mars * t + np.pi / 4),
        r_mars * np.sin(w_mars * t + np.pi / 4),
        np.zeros_like(t),
    )
)


def get_hohmann_pos(r1, r2, theta_start, t_transfer, t_offset, t_all):
    a = (r1 + r2) / 2
    e = abs(r2 - r1) / (r1 + r2)
    n = np.sqrt(1 / a**3)
    M = n * (t_all - t_offset)
    E = M
    r = a * (1 - e**2) / (1 + e * np.cos(E))
    theta = theta_start + np.pi * ((t_all - t_offset) / t_transfer)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.zeros_like(x)
    return np.stack([x, y, z], axis=1)


# Key times & indices
t_depart_idx = 0
transfer_start = t[t_depart_idx]
t_arrive_mars = transfer_start + T_transfer
arrive_mars_idx = np.searchsorted(t, t_arrive_mars)
wait_time = 0.6  # years, approx for window
t_depart_mars = t_arrive_mars + wait_time
depart_mars_idx = np.searchsorted(t, t_depart_mars)
t_arrive_earth = t_depart_mars + T_transfer
arrive_earth_idx = np.searchsorted(t, t_arrive_earth)

sc_pos = np.zeros((n_frames, 3))
sc_pos[:arrive_mars_idx] = get_hohmann_pos(
    r_earth, r_mars, 0, T_transfer, 0, t[:arrive_mars_idx]
)
sc_pos[arrive_mars_idx:depart_mars_idx] = mars_pos[arrive_mars_idx:depart_mars_idx]
theta_start = np.arctan2(mars_pos[depart_mars_idx, 1], mars_pos[depart_mars_idx, 0])
sc_pos[depart_mars_idx:arrive_earth_idx] = get_hohmann_pos(
    r_mars,
    r_earth,
    theta_start,
    T_transfer,
    t[depart_mars_idx],
    t[depart_mars_idx:arrive_earth_idx],
)
sc_pos[arrive_earth_idx:] = earth_pos[arrive_earth_idx:]

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
theta = np.linspace(0, 2 * np.pi, 200)
ax.plot(
    r_earth * np.cos(theta),
    r_earth * np.sin(theta),
    0,
    "b",
    alpha=0.3,
    label="Earth Orbit",
)
ax.plot(
    r_mars * np.cos(theta),
    r_mars * np.sin(theta),
    0,
    "r",
    alpha=0.3,
    label="Mars Orbit",
)
(earth_pt,) = ax.plot([], [], [], "bo", label="Earth")
(mars_pt,) = ax.plot([], [], [], "ro", label="Mars")
(sc_pt,) = ax.plot([], [], [], "ko", label="Spacecraft")
(traj_line,) = ax.plot([], [], [], "k--", alpha=0.5, label="Trajectory")
ax.scatter(0, 0, 0, color="y", s=200, marker="*", label="Sun")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X [AU]")
ax.set_ylabel("Y [AU]")
ax.set_zlabel("Z [AU]")
ax.legend(loc="upper right")


def update(i):
    earth_pt.set_data([earth_pos[i, 0]], [earth_pos[i, 1]])
    earth_pt.set_3d_properties([earth_pos[i, 2]])
    mars_pt.set_data([mars_pos[i, 0]], [mars_pos[i, 1]])
    mars_pt.set_3d_properties([mars_pos[i, 2]])
    sc_pt.set_data([sc_pos[i, 0]], [sc_pos[i, 1]])
    sc_pt.set_3d_properties([sc_pos[i, 2]])
    traj_line.set_data(sc_pos[: i + 1, 0], sc_pos[: i + 1, 1])
    traj_line.set_3d_properties(sc_pos[: i + 1, 2])
    ax.set_title(f"Time: {t[i]:.2f} yr")
    return earth_pt, mars_pt, sc_pt, traj_line


ani = FuncAnimation(fig, update, frames=n_frames, interval=30, blit=True)
plt.show()
