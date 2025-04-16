import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# ---- Simulation parameters ----
# Orbital radii in AU
r_E = 1.0
r_M = 1.524

# Orbital periods in years
T_E = 1.0
T_M = 1.88

# Angular speeds (rad/yr)
w_E = 2 * np.pi / T_E
w_M = 2 * np.pi / T_M

# Hohmann transfer semi‐major axis and eccentricities
a_t = (r_E + r_M) / 2.0
e_out = (r_M - r_E) / (r_M + r_E)  # for Earth→Mars
# time to go Earth→Mars
t_transfer = np.pi * np.sqrt(a_t**3) / np.sqrt(1.0)
# (we've normalized G*M_sun = 1 in AU^3/yr^2 units)

# Synodic period ≈ 1 / |1/T_E - 1/T_M|
T_syn = 1.0 / abs(1 / T_E - 1 / T_M)
# We'll wait until roughly one synodic period after arrival
t_wait = T_syn - t_transfer

# Total simulation time (years)
t_total = t_transfer + t_wait + t_transfer

# Time‐array
n_frames = 800
t_array = np.linspace(0, t_total, n_frames)

# Precompute planet positions for each t
theta_E = w_E * t_array
theta_M = w_M * t_array
x_E = r_E * np.cos(theta_E)
y_E = r_E * np.sin(theta_E)
x_M = r_M * np.cos(theta_M)
y_M = r_M * np.sin(theta_M)

# Precompute ship trajectory
x_ship = np.zeros_like(t_array)
y_ship = np.zeros_like(t_array)

for i, t in enumerate(t_array):
    if t <= t_transfer:
        # outbound transfer Earth->Mars
        phi = (t / t_transfer) * np.pi
        r = a_t * (1 - e_out**2) / (1 + e_out * np.cos(phi))
        # rotate ellipse to Earth's instantaneous position at t=0
        theta0 = 0.0  # we assume Earth starts at angle = 0
        ship_angle = phi + theta0
        x_ship[i] = r * np.cos(ship_angle)
        y_ship[i] = r * np.sin(ship_angle)
    elif t <= t_transfer + t_wait:
        # coasting on/around Mars
        x_ship[i] = r_M * np.cos(w_M * t)
        y_ship[i] = r_M * np.sin(w_M * t)
    else:
        # inbound transfer Mars->Earth
        t2 = t - (t_transfer + t_wait)
        phi2 = (t2 / t_transfer) * np.pi
        # eccentricity for return leg (periapsis at Mars orbit)
        e_in = e_out
        a2 = a_t
        # use r(φ) = a(1−e^2)/(1 − e cosφ) => φ=0 gives r=r_M, φ=π gives r=r_E
        r2 = a2 * (1 - e_in**2) / (1 - e_in * np.cos(phi2))
        # rotate by Mars departure angle
        theta_depart = w_M * (t_transfer + t_wait)
        ship_angle = phi2 + theta_depart
        x_ship[i] = r2 * np.cos(ship_angle)
        y_ship[i] = r2 * np.sin(ship_angle)

# Set up 3D plot (all orbits in the ecliptic plane z=0)
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1.7, 1.7)
ax.set_ylim(-1.7, 1.7)
ax.set_zlim(-0.2, 0.2)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth→Mars→Earth Mission Profile")

# Draw the Sun
(sun,) = ax.plot([0], [0], [0], "o", color="yellow", markersize=12, label="Sun")

# Pre‐plot the circular orbits
t_orb = np.linspace(0, 2 * np.pi, 200)
ax.plot(
    r_E * np.cos(t_orb),
    r_E * np.sin(t_orb),
    0 * t_orb,
    color="blue",
    label="Earth Orbit",
)
ax.plot(
    r_M * np.cos(t_orb), r_M * np.sin(t_orb), 0 * t_orb, color="red", label="Mars Orbit"
)

# Create scatter objects for animating
(pt_Earth,) = ax.plot([], [], [], "o", color="blue", markersize=6, label="Earth")
(pt_Mars,) = ax.plot([], [], [], "o", color="red", markersize=6, label="Mars")
(pt_ship,) = ax.plot([], [], [], "o", color="black", markersize=4, label="Ship")

ax.legend(loc="upper right")


def init():
    pt_Earth.set_data([], [])
    pt_Earth.set_3d_properties([])
    pt_Mars.set_data([], [])
    pt_Mars.set_3d_properties([])
    pt_ship.set_data([], [])
    pt_ship.set_3d_properties([])
    return pt_Earth, pt_Mars, pt_ship


def update(i):
    # update Earth
    pt_Earth.set_data(x_E[i], y_E[i])
    pt_Earth.set_3d_properties(0)
    # update Mars
    pt_Mars.set_data(x_M[i], y_M[i])
    pt_Mars.set_3d_properties(0)
    # update spacecraft
    pt_ship.set_data(x_ship[i], y_ship[i])
    pt_ship.set_3d_properties(0)
    return pt_Earth, pt_Mars, pt_ship


ani = FuncAnimation(
    fig, update, frames=n_frames, init_func=init, blit=True, interval=30
)

plt.show()
