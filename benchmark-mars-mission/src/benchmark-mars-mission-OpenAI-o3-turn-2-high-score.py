# launch_to_mars_and_back.py
"""
Animated 3‑D trajectory Earth ➞ Mars ➞ Earth (next launch window).

Dependencies:
    numpy
    matplotlib>=3.6  (for animation and %matplotlib notebook or video saving)

Run:
    python launch_to_mars_and_back.py
"""
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3‑D projection)
from matplotlib.animation import FuncAnimation

# ----------------------------
# Orbital parameters (mean values, SI units)
# ----------------------------
AU = 1.496e11  # Astronomical Unit [m]
YEAR = 365.25 * 24 * 3600  # Earth orbital period [s]
DAY = 24 * 3600  # seconds in one day

R_EARTH = 1.0 * AU  # Earth semi‑major axis
R_MARS = 1.524 * AU  # Mars  semi‑major axis
T_EARTH = 1.0 * YEAR
T_MARS = 1.881 * YEAR

# Transfer parameters（霍曼转移）
transfer_time_em = 259 * DAY  # Earth➞Mars flight time ~259d
transfer_time_me = 259 * DAY  # Mars➞Earth flight time
surface_stay = 60 * DAY  # Stay on Mars 60d


# ----------------------------
# Utility functions
# ----------------------------
def circular_orbit(radius, period, t):
    """Return x, y, z of a planet on circular orbit in ecliptic plane."""
    theta = 2 * np.pi * (t / period)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    z = 0.0
    return x, y, z


def hohmann_transfer(r1, r2, t, tof):
    """
    Position along a Hohmann transfer ellipse from r1 (periapsis) to r2 (apoapsis).
    Args:
        r1, r2 : peri/apoapsis distance
        t      : current time since departure [s]
        tof    : total time of flight [s]
    """
    a = 0.5 * (r1 + r2)  # semi‑major axis
    e = 1 - r1 / a  # eccentricity
    # Mean anomaly M
    M = np.pi * t / tof  # linear w.r.t time for Keplerian approx
    # Solve Kepler eqn E - e*sinE = M (Newton–Raphson)
    E = M
    for _ in range(5):  # few iterations sufficient
        E = E - (E - e * np.sin(E) - M) / (1 - e * np.cos(E))
    # True anomaly ν
    nu = 2 * np.arctan2(np.sqrt(1 + e) * np.sin(E / 2), np.sqrt(1 - e) * np.cos(E / 2))
    # Radius in polar coordinates
    r = a * (1 - e * np.cos(E))
    x = r * np.cos(nu)
    y = r * np.sin(nu)
    z = 0.0
    return x, y, z


# ----------------------------
# Time line
# ----------------------------
t_depart_E = 0.0
t_arrive_M = t_depart_E + transfer_time_em
t_depart_M = t_arrive_M + surface_stay
t_arrive_E = t_depart_M + transfer_time_me

# Build overall timeline for animation (step 1 day)
t_span = np.arange(0, t_arrive_E + 1 * DAY, 1 * DAY)

# Pre‑compute trajectories for planets
earth_traj = np.array([circular_orbit(R_EARTH, T_EARTH, t) for t in t_span])
mars_traj = np.array([circular_orbit(R_MARS, T_MARS, t) for t in t_span])

# Pre‑compute spacecraft trajectory (piece‑wise)
sc_traj = []
for t in t_span:
    if t < t_arrive_M:  # leg 1: E➞M
        sc_traj.append(
            hohmann_transfer(R_EARTH, R_MARS, t - t_depart_E, transfer_time_em)
        )
    elif t < t_depart_M:  # stay on Mars
        idx = int(t_arrive_M / DAY)  # hold Mars pos
        sc_traj.append(mars_traj[idx])
    else:  # leg 2: M➞E
        sc_traj.append(
            hohmann_transfer(R_MARS, R_EARTH, t - t_depart_M, transfer_time_me)
        )
sc_traj = np.array(sc_traj)

# ----------------------------
# Animation setup
# ----------------------------
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_facecolor("k")
ax.set_xlim(-1.7 * AU, 1.7 * AU)
ax.set_ylim(-1.7 * AU, 1.7 * AU)
ax.set_zlim(-1.7 * AU * 0.05, 1.7 * AU * 0.05)  # slight thickness
ax.set_xticks([])
ax.set_yticks([])
ax.set_zticks([])
ax.set_title("Earth ↔ Mars Transfer", color="w")
for spine in ax.spines.values():
    spine.set_color("w")
fig.patch.set_facecolor("k")

# Plot static orbits
theta = np.linspace(0, 2 * np.pi, 200)
ax.plot(
    R_EARTH * np.cos(theta),
    R_EARTH * np.sin(theta),
    0,
    color="deepskyblue",
    linewidth=0.7,
    label="Earth Orbit",
)
ax.plot(
    R_MARS * np.cos(theta),
    R_MARS * np.sin(theta),
    0,
    color="orangered",
    linewidth=0.7,
    label="Mars Orbit",
)
ax.scatter(0, 0, 0, color="yellow", s=150, label="Sun")

# Initialize scatter objects
planet_E = ax.scatter([], [], [], color="deepskyblue", s=50)
planet_M = ax.scatter([], [], [], color="orangered", s=50)
craft = ax.scatter([], [], [], color="white", s=20)

# Text annotation
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes, color="w")


def init():
    """Clear scatters before animation starts."""
    planet_E._offsets3d = ([], [], [])
    planet_M._offsets3d = ([], [], [])
    craft._offsets3d = ([], [], [])
    time_text.set_text("")
    return planet_E, planet_M, craft, time_text


def update(frame):
    """Update positions for frame index."""
    xe, ye, ze = earth_traj[frame]
    xm, ym, zm = mars_traj[frame]
    xs, ys, zs = sc_traj[frame]

    planet_E._offsets3d = ([xe], [ye], [ze])
    planet_M._offsets3d = ([xm], [ym], [zm])
    craft._offsets3d = ([xs], [ys], [zs])

    # Update time annotation (in days)
    days = int(t_span[frame] / DAY)
    time_text.set_text(f"Day {days}")

    return planet_E, planet_M, craft, time_text


ani = FuncAnimation(
    fig, update, frames=len(t_span), init_func=init, interval=20, blit=True
)

plt.legend(facecolor="k", edgecolor="w", labelcolor="w")
plt.show()
