"""
3D animation: Earth‑to‑Mars Hohmann transfer and return flight.

Author: ChatGPT demo
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
from matplotlib.animation import FuncAnimation

# -----------------------------
# Constants (simplified)
# -----------------------------
AU = 1.0  # Astronomical Unit in our scale
R_EARTH = 1.0 * AU  # Earth orbital radius
R_MARS = 1.52 * AU  # Mars orbital radius
T_EARTH = 1.0  # Earth orbital period (year)
T_MARS = 1.88  # Mars orbital period (year)

# Hohmann transfer (Earth → Mars and Mars → Earth use same a,e)
a_transfer = (R_EARTH + R_MARS) / 2.0  # semi‑major axis
e_transfer = (R_MARS - R_EARTH) / (R_MARS + R_EARTH)  # eccentricity
t_transfer = np.pi * np.sqrt(a_transfer**3)  # half orbital period (in years)

# Time line (years) --------------------------------------------------
stop_over = 0.5  # stay on Mars (years) ~ 6 months
t0_launch = 0.0
t1_arriveMars = t0_launch + t_transfer
t2_departMars = t1_arriveMars + stop_over
t3_arriveEarth = t2_departMars + t_transfer
total_time = t3_arriveEarth

# simulation resolution
N_frames = 1000
times = np.linspace(0.0, total_time, N_frames)


# -------------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------------
def planet_pos(radius, period, t):
    """Return Cartesian position of a planet in ecliptic plane (z=0)."""
    theta = 2 * np.pi * t / period
    return radius * np.cos(theta), radius * np.sin(theta), 0.0


def transfer_pos(radius_start, radius_end, t_local, direction=+1):
    """
    Position on Hohmann transfer ellipse from radius_start to radius_end.

    direction = +1 for outbound (Earth→Mars), ‑1 for inbound (Mars→Earth)
    t_local   = time since departure (0 … t_transfer)
    """
    # true anomaly ranges 0→π (outbound) or π→2π (inbound)
    if direction == +1:
        true_anom = np.pi * t_local / t_transfer
    else:
        true_anom = np.pi + np.pi * t_local / t_transfer

    # polar equation of ellipse (focus at Sun)
    r = (a_transfer * (1 - e_transfer**2)) / (
        1 + e_transfer * np.cos(true_anom - (0 if direction == +1 else np.pi))
    )
    x = r * np.cos(true_anom)
    y = r * np.sin(true_anom)
    return x, y, 0.0


# Pre‑compute trajectories for animation --------------------------------
earth_xyz = np.zeros((N_frames, 3))
mars_xyz = np.zeros((N_frames, 3))
probe_xyz = np.zeros((N_frames, 3)) * np.nan  # NaN until launched

for i, t in enumerate(times):
    # Planets
    earth_xyz[i] = planet_pos(R_EARTH, T_EARTH, t)
    mars_xyz[i] = planet_pos(R_MARS, T_MARS, t)

    # Probe
    if t < t1_arriveMars:
        # outbound leg
        probe_xyz[i] = transfer_pos(R_EARTH, R_MARS, t - t0_launch, direction=+1)
    elif t < t2_departMars:
        # waiting at Mars
        probe_xyz[i] = mars_xyz[i]
    elif t < t3_arriveEarth:
        # return leg
        probe_xyz[i] = transfer_pos(R_MARS, R_EARTH, t - t2_departMars, direction=-1)
    else:
        # after arrival remain with Earth
        probe_xyz[i] = earth_xyz[i]

# -------------------------------------------------------------------
# Matplotlib 3D animation set‑up
# -------------------------------------------------------------------
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("k")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.2, 0.2)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth‑Mars Transfer and Return", color="w")
for spine in ax.spines.values():
    spine.set_color("w")
ax.xaxis.label.set_color("w")
ax.yaxis.label.set_color("w")
ax.zaxis.label.set_color("w")
ax.tick_params(colors="w")

# Static artists: Sun, circular orbits
(sun,) = ax.plot([0], [0], [0], marker="o", markersize=10, color="yellow", label="Sun")

theta_full = np.linspace(0, 2 * np.pi, 400)
ax.plot(
    R_EARTH * np.cos(theta_full),
    R_EARTH * np.sin(theta_full),
    0,
    color="deepskyblue",
    alpha=0.3,
)
ax.plot(
    R_MARS * np.cos(theta_full),
    R_MARS * np.sin(theta_full),
    0,
    color="orange",
    alpha=0.3,
)

# Dynamic artists
(earth_dot,) = ax.plot([], [], [], "o", color="deepskyblue", label="Earth")
(mars_dot,) = ax.plot([], [], [], "o", color="orange", label="Mars")
(probe_dot,) = ax.plot([], [], [], "o", color="white", label="Probe")
(probe_path,) = ax.plot([], [], [], color="white", lw=1, alpha=0.6)

ax.legend(facecolor="k", framealpha=0.2, labelcolor="w")


# Animation function --------------------------------------------------
def init():
    earth_dot.set_data_3d([], [], [])
    mars_dot.set_data_3d([], [], [])
    probe_dot.set_data_3d([], [], [])
    probe_path.set_data_3d([], [], [])
    return earth_dot, mars_dot, probe_dot, probe_path


def update(frame):
    # update planets
    earth_dot.set_data_3d(*earth_xyz[frame].T)
    mars_dot.set_data_3d(*mars_xyz[frame].T)

    # update probe
    probe_dot.set_data_3d(*probe_xyz[frame].T)
    path_slice = probe_xyz[: frame + 1]
    probe_path.set_data_3d(path_slice[:, 0], path_slice[:, 1], path_slice[:, 2])
    return earth_dot, mars_dot, probe_dot, probe_path


ani = FuncAnimation(
    fig, update, frames=N_frames, init_func=init, interval=30, blit=True
)

plt.show()
