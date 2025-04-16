import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# 1) PARAMETERS & ORBITS ------------------------------------------------

# Orbital radii in AU
r_earth = 1.0
r_mars = 1.524

# Mean motions (rad/day)
w_earth = 2 * np.pi / 365.25
w_mars = 2 * np.pi / 687.0

# Hohmann‐style transfer ellipse parameters
a = 0.5 * (r_earth + r_mars)  # semi‐major axis
e = (r_mars - r_earth) / (r_mars + r_earth)

# Approximate durations (days)
t_dep_to_arr = 259  # Earth → Mars
t_wait_at_mars = 600  # loiter time on Mars
t_ret_to_arr = 259  # Mars → Earth
T_total = int(t_dep_to_arr + t_wait_at_mars + t_ret_to_arr)

# Time array
times = np.arange(0, T_total + 1)  # days

# Precompute Earth & Mars positions
pos_e = np.vstack(
    [
        r_earth * np.cos(w_earth * times),
        r_earth * np.sin(w_earth * times),
        np.zeros_like(times),
    ]
)

pos_m = np.vstack(
    [
        r_mars * np.cos(w_mars * times),
        r_mars * np.sin(w_mars * times),
        np.zeros_like(times),
    ]
)


# 2) BUILD TRANSFER ELLIPSES --------------------------------------------

N_pts = 500

# departure ellipse (ν from 0→π)
nu1 = np.linspace(0, np.pi, N_pts)
r1 = a * (1 - e**2) / (1 + e * np.cos(nu1))
# start‐ellipse at Earth's departure angle = 0
x1 = r1 * np.cos(nu1)
y1 = r1 * np.sin(nu1)

# return ellipse (ν from π→2π), rotated so it departs Mars at the
# correct Mars‐angle at t = t_dep_to_arr + t_wait_at_mars
t2 = t_dep_to_arr + t_wait_at_mars
theta_m2 = w_mars * t2
nu2 = np.linspace(np.pi, 2 * np.pi, N_pts)
r2 = a * (1 - e**2) / (1 + e * np.cos(nu2))
# we want ν=π to lie at θ=theta_m2, so offset = theta_m2 - π
offset = theta_m2 - np.pi
x2 = r2 * np.cos(nu2 + offset)
y2 = r2 * np.sin(nu2 + offset)


# 3) ASSEMBLE SPACECRAFT TRACK ------------------------------------------

sc = np.zeros((3, T_total + 1))
for t in times:
    if t <= t_dep_to_arr:
        # outgoing
        idx = int((t / t_dep_to_arr) * (N_pts - 1))
        sc[:, t] = [x1[idx], y1[idx], 0.0]
    elif t <= t_dep_to_arr + t_wait_at_mars:
        # parked at Mars
        sc[:, t] = pos_m[:, t]
    else:
        # return
        tt = t - (t_dep_to_arr + t_wait_at_mars)
        idx = int((tt / t_ret_to_arr) * (N_pts - 1))
        sc[:, t] = [x2[idx], y2[idx], 0.0]


# 4) SET UP THE MATPLOTLIB 3D ANIMATION ----------------------------------

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")

# draw Sun
ax.scatter([0], [0], [0], color="gold", s=100, label="Sun")

# draw orbits
theta = np.linspace(0, 2 * np.pi, 1000)
ax.plot(
    r_earth * np.cos(theta),
    r_earth * np.sin(theta),
    0,
    color="blue",
    lw=1,
    label="Earth Orbit",
)
ax.plot(
    r_mars * np.cos(theta),
    r_mars * np.sin(theta),
    0,
    color="red",
    lw=1,
    label="Mars Orbit",
)

# draw the 2 transfer‐ellipses for reference
ax.plot(x1, y1, 0, color="green", linestyle="--", lw=0.5)
ax.plot(x2, y2, 0, color="green", linestyle="--", lw=0.5)

# dynamic points
(p_e,) = ax.plot([], [], [], "o", color="blue", markersize=6, label="Earth")
(p_m,) = ax.plot([], [], [], "o", color="red", markersize=6, label="Mars")
(p_s,) = ax.plot([], [], [], "o", color="black", markersize=4, label="Spacecraft")

ax.set_xlim(-1.7, 1.7)
ax.set_ylim(-1.7, 1.7)
ax.set_zlim(-0.1, 0.1)
ax.set_xlabel("x (AU)")
ax.set_ylabel("y (AU)")
ax.set_zlabel("z (AU)")
ax.set_title("Earth→Mars→Earth Round‐Trip")
ax.legend()


def update(n):
    # Earth
    p_e.set_data(pos_e[0, n], pos_e[1, n])
    p_e.set_3d_properties(pos_e[2, n])
    # Mars
    p_m.set_data(pos_m[0, n], pos_m[1, n])
    p_m.set_3d_properties(pos_m[2, n])
    # Spacecraft
    p_s.set_data(sc[0, n], sc[1, n])
    p_s.set_3d_properties(sc[2, n])
    return p_e, p_m, p_s


anim = FuncAnimation(fig, update, frames=T_total + 1, interval=30, blit=True)

plt.show()
