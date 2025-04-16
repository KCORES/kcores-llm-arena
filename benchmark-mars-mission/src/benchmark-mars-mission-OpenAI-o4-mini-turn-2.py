import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ---- Mission & orbital parameters ----
# Astronomical unit [AU], time in years
r_e = 1.0  # Earth's orbital radius [AU]
r_m = 1.524  # Mars' orbital radius [AU]
T_e = 1.0  # Earth's period [yr]
T_m = 1.8808  # Mars' period [yr]
mu = 4 * np.pi**2  # Gravitational parameter, so that Earth period=1 yr

# Hohmann‐transfer ellipse parameters
a_trans = (r_e + r_m) / 2.0
e_trans = (r_m - r_e) / (r_m + r_e)
# half‑period of that ellipse:
T_transfer = 0.5 * np.sqrt(a_trans**3 / (mu / (4 * np.pi**2)))  # ~0.71 yr

# synodic period for Earth‐Mars
T_synodic = 1.0 / abs(1 / T_e - 1 / T_m)

# departure and arrival times
t0 = 0.0
t_arrive_m = t0 + T_transfer
t_depart_m = t0 + T_synodic  # launch window for return
t_return = t_depart_m + T_transfer
t_total = t_return

# number of frames
n_frames = 400

# precompute time array
t = np.linspace(0, t_total, n_frames)

# compute planet angles
theta_e = 2 * np.pi * t / T_e
theta_m = 2 * np.pi * t / T_m

# Earth & Mars positions
x_e = r_e * np.cos(theta_e)
y_e = r_e * np.sin(theta_e)
z_e = np.zeros_like(x_e)
x_m = r_m * np.cos(theta_m)
y_m = r_m * np.sin(theta_m)
z_m = np.zeros_like(x_m)

# spacecraft trajectory
x_sc = np.zeros_like(t)
y_sc = np.zeros_like(t)
z_sc = np.zeros_like(t)

# Orientation offset for first transfer: launch from Earth's position at t0
theta0_e = theta_e[0]  # typically zero

# Orientation offset for return transfer so that departure aligns with Mars pos
theta_dep_m = np.interp(t_depart_m, t, theta_m)
offset_return = np.pi - theta_dep_m

for i, ti in enumerate(t):
    if ti <= t_arrive_m:
        # outbound transfer: phi runs from 0 to pi
        phi = np.pi * (ti - t0) / T_transfer
        rphi = a_trans * (1 - e_trans**2) / (1 + e_trans * np.cos(phi))
        ang = phi + theta0_e
        x_sc[i] = rphi * np.cos(ang)
        y_sc[i] = rphi * np.sin(ang)
    elif ti < t_depart_m:
        # parked at Mars until next window
        x_sc[i] = x_m[i]
        y_sc[i] = y_m[i]
    else:
        # return transfer: phi runs from pi to 2pi
        phi_r = np.pi + np.pi * (ti - t_depart_m) / T_transfer
        rphi = a_trans * (1 - e_trans**2) / (1 + e_trans * np.cos(phi_r))
        ang = phi_r + offset_return
        x_sc[i] = rphi * np.cos(ang)
        y_sc[i] = rphi * np.sin(ang)

# ---- set up matplotlib 3D animation ----
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1.7, 1.7)
ax.set_ylim(-1.7, 1.7)
ax.set_zlim(-0.3, 0.3)
ax.set_xlabel("X [AU]")
ax.set_ylabel("Y [AU]")
ax.set_zlabel("Z [AU]")
ax.set_title("Earth→Mars→Earth Hohmann Transfers")

# plot Sun
(sun,) = ax.plot([0], [0], [0], marker="o", color="yellow", markersize=12, label="Sun")
# plot orbits
ax.plot(
    r_e * np.cos(np.linspace(0, 2 * np.pi, 200)),
    r_e * np.sin(np.linspace(0, 2 * np.pi, 200)),
    0,
    color="blue",
    alpha=0.3,
)
ax.plot(
    r_m * np.cos(np.linspace(0, 2 * np.pi, 200)),
    r_m * np.sin(np.linspace(0, 2 * np.pi, 200)),
    0,
    color="red",
    alpha=0.3,
)

# scatter objects
(earth_scatter,) = ax.plot([], [], [], "o", color="blue", markersize=6, label="Earth")
(mars_scatter,) = ax.plot([], [], [], "o", color="red", markersize=6, label="Mars")
(sc_scatter,) = ax.plot(
    [], [], [], "o", color="black", markersize=4, label="Spacecraft"
)

# trails
(trail_e,) = ax.plot([], [], [], "-", color="blue", linewidth=1, alpha=0.4)
(trail_m,) = ax.plot([], [], [], "-", color="red", linewidth=1, alpha=0.4)
(trail_sc,) = ax.plot([], [], [], "-", color="black", linewidth=1, alpha=0.7)

ax.legend(loc="upper right")


def init():
    earth_scatter.set_data([], [])
    earth_scatter.set_3d_properties([])
    mars_scatter.set_data([], [])
    mars_scatter.set_3d_properties([])
    sc_scatter.set_data([], [])
    sc_scatter.set_3d_properties([])
    trail_e.set_data([], [])
    trail_e.set_3d_properties([])
    trail_m.set_data([], [])
    trail_m.set_3d_properties([])
    trail_sc.set_data([], [])
    trail_sc.set_3d_properties([])
    return earth_scatter, mars_scatter, sc_scatter, trail_e, trail_m, trail_sc


def update(frame):
    # planets
    earth_scatter.set_data(x_e[frame], y_e[frame])
    earth_scatter.set_3d_properties(z_e[frame])
    mars_scatter.set_data(x_m[frame], y_m[frame])
    mars_scatter.set_3d_properties(z_m[frame])
    # spacecraft
    sc_scatter.set_data(x_sc[frame], y_sc[frame])
    sc_scatter.set_3d_properties(0.0)
    # trails up to this frame
    trail_e.set_data(x_e[:frame], y_e[:frame])
    trail_e.set_3d_properties(z_e[:frame])
    trail_m.set_data(x_m[:frame], y_m[:frame])
    trail_m.set_3d_properties(z_m[:frame])
    trail_sc.set_data(x_sc[:frame], y_sc[:frame])
    trail_sc.set_3d_properties(np.zeros(frame))
    return earth_scatter, mars_scatter, sc_scatter, trail_e, trail_m, trail_sc


anim = FuncAnimation(
    fig, update, frames=n_frames, init_func=init, blit=True, interval=30
)

# to save as mp4, uncomment below (requires ffmpeg):
# anim.save('earth_mars_return.mp4', fps=25, dpi=200)

plt.show()
