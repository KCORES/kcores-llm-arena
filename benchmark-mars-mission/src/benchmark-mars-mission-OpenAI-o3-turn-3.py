# animated_earth_mars_transfer.py
"""
Animated 3‑D plot showing a spacecraft launching from Earth,
performing a Hohmann transfer to Mars, waiting for the next
launch window, then transferring back to Earth.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401, needed for 3‑D plots
from matplotlib.animation import FuncAnimation

# ----------------------------- Constants -----------------------------
AU = 1.0  # Astronomical units for distance
DEG2RAD = np.pi / 180.0
# Gravitational parameter of the Sun in AU^3 / year^2
MU_SUN = 4.0 * np.pi**2

# Orbital radii (assumed circular for simplicity)
R_EARTH = 1.0 * AU
R_MARS = 1.523 * AU

# Orbital periods derived from Kepler's 3rd law (T^2 = (4π²/μ) a^3)
T_EARTH = 2.0 * np.pi * np.sqrt(R_EARTH**3 / MU_SUN)  # ≈ 1 year
T_MARS = 2.0 * np.pi * np.sqrt(R_MARS**3 / MU_SUN)  # ≈ 1.88 years

# Hohmann transfer parameters
a_transfer = 0.5 * (R_EARTH + R_MARS)  # semi‑major axis
e_transfer = (R_MARS - R_EARTH) / (R_MARS + R_EARTH)  # eccentricity
T_TRANSFER = np.pi * np.sqrt(
    a_transfer**3 / MU_SUN
)  # half‑ellipse flight time (~259 days ≈ 0.709 yr)

WAIT_AT_MARS = 0.5  # years to wait before return launch (approx.)
T_TOTAL = 3.0  # total simulated time (years)
DT = 0.01  # simulation step (years) ≈ 3.65 days

# Mars orbital inclination (for 3‑D effect only)
INC_MARS = 1.85 * DEG2RAD


# ------------------------ Helper functions ---------------------------
def planet_position(radius, period, t, inclination=0.0):
    """Return heliocentric planet position at time t (in years)."""
    theta = 2.0 * np.pi * t / period
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = radius * np.sin(inclination) * np.sin(theta)
    return np.array([x, y, z])


def transfer_position(t_local, outward=True):
    """
    Position on a Hohmann transfer ellipse.
    t_local runs 0 → T_TRANSFER (or vice‑versa).
    """
    # Mean anomaly: M = n * t
    n = np.sqrt(MU_SUN / a_transfer**3)
    M = n * t_local
    # Solve Kepler's equation for E (eccentric anomaly) via Newton–Raphson
    E = M.copy()
    for _ in range(5):
        E = E - (E - e_transfer * np.sin(E) - M) / (1 - e_transfer * np.cos(E))
    # True anomaly
    theta = 2.0 * np.arctan2(
        np.sqrt(1 + e_transfer) * np.sin(E / 2), np.sqrt(1 - e_transfer) * np.cos(E / 2)
    )
    if not outward:  # return trip travels second half of ellipse
        theta = np.pi - theta
    r = a_transfer * (1 - e_transfer * np.cos(E))
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = 0.0
    return np.array([x, y, z])


# ------------------------ Pre‑compute ephemeris ----------------------
times = np.arange(0, T_TOTAL, DT)
earth_xyz = np.array([planet_position(R_EARTH, T_EARTH, t) for t in times])
mars_xyz = np.array([planet_position(R_MARS, T_MARS, t, INC_MARS) for t in times])

# Spacecraft timeline
launch_out = 0.0
arrive_mars = launch_out + T_TRANSFER
launch_back = arrive_mars + WAIT_AT_MARS
arrive_home = launch_back + T_TRANSFER

craft_xyz = np.zeros_like(earth_xyz)

for i, t in enumerate(times):
    if launch_out <= t < arrive_mars:
        craft_xyz[i] = transfer_position(t - launch_out, outward=True)
    elif arrive_mars <= t < launch_back:
        craft_xyz[i] = mars_xyz[i]  # stay with Mars
    elif launch_back <= t < arrive_home:
        # run transfer backwards (mirror time)
        craft_xyz[i] = transfer_position(t - launch_back, outward=False)
    else:
        craft_xyz[i] = earth_xyz[i]  # sitting at Earth (initial/final)

# ---------------------------- Plot setup -----------------------------
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("black")
fig.patch.set_facecolor("black")

# Orbital paths (static)
ax.plot(
    earth_xyz[:, 0],
    earth_xyz[:, 1],
    earth_xyz[:, 2],
    color="deepskyblue",
    alpha=0.3,
    label="Earth orbit",
)
ax.plot(
    mars_xyz[:, 0],
    mars_xyz[:, 1],
    mars_xyz[:, 2],
    color="orangered",
    alpha=0.3,
    label="Mars orbit",
)

# Scatter objects (animated)
(earth_scat,) = ax.plot(
    [], [], [], "o", color="deepskyblue", markersize=8, label="Earth"
)
(mars_scat,) = ax.plot([], [], [], "o", color="orangered", markersize=6, label="Mars")
(craft_scat,) = ax.plot(
    [], [], [], "o", color="white", markersize=4, label="Spacecraft"
)

# Aesthetics
ax.legend(loc="upper left", fontsize=9)
ax.set_xlim(-1.7, 1.7)
ax.set_ylim(-1.7, 1.7)
ax.set_zlim(-0.2, 0.2)
ax.set_xlabel("X (AU)", color="white")
ax.set_ylabel("Y (AU)", color="white")
ax.set_zlabel("Z (AU)", color="white")
for spine in (
    ax.w_xaxis.get_ticklines() + ax.w_yaxis.get_ticklines() + ax.w_zaxis.get_ticklines()
):
    spine.set_color("white")
for axis in [ax.w_xaxis, ax.w_yaxis, ax.w_zaxis]:
    axis._axinfo["grid"]["color"] = "gray"


# --------------------------- Animation loop --------------------------
def update(frame):
    earth_scat.set_data(earth_xyz[frame, 0], earth_xyz[frame, 1])
    earth_scat.set_3d_properties(earth_xyz[frame, 2])

    mars_scat.set_data(mars_xyz[frame, 0], mars_xyz[frame, 1])
    mars_scat.set_3d_properties(mars_xyz[frame, 2])

    craft_scat.set_data(craft_xyz[frame, 0], craft_xyz[frame, 1])
    craft_scat.set_3d_properties(craft_xyz[frame, 2])
    return earth_scat, mars_scat, craft_scat


ani = FuncAnimation(fig, update, frames=len(times), interval=20, blit=True)
plt.show()
