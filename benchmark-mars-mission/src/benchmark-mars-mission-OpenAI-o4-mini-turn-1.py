import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# 1) orbital constants (in AU and years)
re = 1.0  # Earth orbit radius [AU]
rm = 1.524  # Mars orbit radius [AU]
Te = 1.0  # Earth period [yr]
Tm = rm**1.5  # Mars period [yr] by Kepler's 3rd law: T^2=a^3
we = 2 * np.pi / Te  # Earth's angular speed [rad/yr]
wm = 2 * np.pi / Tm  # Mars' angular speed

# 2) Hohmann‐transfer semi‐major & eccentricity
a = 0.5 * (re + rm)
e = (rm - re) / (rm + re)
t_trans = 0.5 * a**1.5  # transfer half‐period [yr]

# 3) times (years)
t1 = t_trans  # Earth->Mars transfer time
t2 = 1.0 / (1 / Te - 1 / Tm)  # next launch window ~synodic period
t3 = t2 + t_trans  # Mars->Earth arrival

# total animation time
t_total = t3
Nf = 500  # number of frames

# 4) time vector
t = np.linspace(0, t_total, Nf)

# 5) precompute planet positions
earth = np.zeros((3, Nf))
mars = np.zeros((3, Nf))
for i, ti in enumerate(t):
    earth[0, i] = re * np.cos(we * ti)
    earth[1, i] = re * np.sin(we * ti)
    mars[0, i] = rm * np.cos(
        wm * ti + (np.pi - wm * t1)
    )  # phase offset so Mars is at transfer arrival

    mars[1, i] = rm * np.sin(wm * ti + (np.pi - wm * t1))
    # z stays zero

# 6) precompute spacecraft trajectory
sc = np.zeros((3, Nf))

# orientation of outbound ellipse (line of apsides at angle θe0=0)
phi0_out = 0.0

# orientation of return ellipse (so that its apogee starts at Mars at t2)
theta_m_t2 = wm * t2 + (np.pi - wm * t1)
phi0_back = theta_m_t2 - np.pi

for i, ti in enumerate(t):
    if ti <= t1:
        # outbound
        theta = np.pi * (ti / t1)  # 0->π
        r = a * (1 - e**2) / (1 + e * np.cos(theta))
        phi = phi0_out + theta
        sc[0, i] = r * np.cos(phi)
        sc[1, i] = r * np.sin(phi)

    elif ti < t2:
        # parked on Mars
        sc[:, i] = mars[:, i]

    else:
        # inbound
        dt = ti - t2
        theta = np.pi + np.pi * (dt / t_trans)  # π->2π
        r = a * (1 - e**2) / (1 + e * np.cos(theta))
        phi = phi0_back + theta
        sc[0, i] = r * np.cos(phi)
        sc[1, i] = r * np.sin(phi)

# 7) build the animation
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1.6, 1.6)
ax.set_ylim(-1.6, 1.6)
ax.set_zlim(-0.5, 0.5)
ax.set_xlabel("X [AU]")
ax.set_ylabel("Y [AU]")
ax.set_zlabel("Z [AU]")
ax.set_title("Earth→Mars→Earth Hohmann‐transfer animation")

# plot orbits
θ = np.linspace(0, 2 * np.pi, 300)
ax.plot(re * np.cos(θ), re * np.sin(θ), 0 * θ, "b", label="Earth orbit")
ax.plot(rm * np.cos(θ), rm * np.sin(θ), 0 * θ, "r", label="Mars orbit")

# path traces
(line_sc,) = ax.plot([], [], [], "k-", lw=1, alpha=0.6)
(pt_sc,) = ax.plot([], [], [], "ko", label="Spacecraft")
(pt_e,) = ax.plot([], [], [], "bo", label="Earth")
(pt_m,) = ax.plot([], [], [], "ro", label="Mars")

ax.legend(loc="upper right")


def init():
    line_sc.set_data([], [])
    line_sc.set_3d_properties([])
    pt_sc.set_data([], [])
    pt_sc.set_3d_properties([])
    pt_e.set_data([], [])
    pt_e.set_3d_properties([])
    pt_m.set_data([], [])
    pt_m.set_3d_properties([])
    return line_sc, pt_sc, pt_e, pt_m


def update(i):
    # spacecraft trace up to i
    line_sc.set_data(sc[0, :i], sc[1, :i])
    line_sc.set_3d_properties(sc[2, :i])

    # current positions
    pt_sc.set_data(sc[0, i], sc[1, i])
    pt_sc.set_3d_properties(sc[2, i])
    pt_e.set_data(earth[0, i], earth[1, i])
    pt_e.set_3d_properties(earth[2, i])
    pt_m.set_data(mars[0, i], mars[1, i])
    pt_m.set_3d_properties(mars[2, i])

    return line_sc, pt_sc, pt_e, pt_m


ani = FuncAnimation(fig, update, frames=Nf, init_func=init, blit=True, interval=30)

plt.show()
