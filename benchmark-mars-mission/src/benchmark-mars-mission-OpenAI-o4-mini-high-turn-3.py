import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 1) PHYSICAL & ORBITAL CONSTANTS (normalized units)
mu = 4 * np.pi**2  # Sun’s gravitational parameter [AU^3/yr^2]
r1 = 1.0  # Earth’s orbital radius [AU]
r2 = 1.524  # Mars’s orbital radius [AU]
T_E = 2 * np.pi * np.sqrt(r1**3 / mu)  # =1 yr
T_M = 2 * np.pi * np.sqrt(r2**3 / mu)
ωE = 2 * np.pi / T_E
ωM = 2 * np.pi / T_M

# 2) HOHMANN TRANSFER PARAMETERS
a = 0.5 * (r1 + r2)  # semimajor axis
e = (r2 - r1) / (r1 + r2)  # eccentricity
tof = np.pi * np.sqrt(a**3 / mu)  # one‐way flight time [yr] ~0.71 yr

# 3) PHASE TIMES (years)
t_out = tof  # Earth→Mars
t_wait = 2.136  # wait on Mars for next window (~synodic period)
t_back = tof  # Mars→Earth

# 4) TIME GRID
N1 = 200
N2 = 200
N3 = 200
t1 = np.linspace(0, t_out, N1)
t2 = np.linspace(0, t_wait, N2)
t3 = np.linspace(0, t_back, N3)
# full timeline
t = np.hstack((t1, t_out + t2, t_out + t_wait + t3))

# 5) INITIAL PHASE ANGLE FOR MARS SO THAT H OHMAN LINES UP
phiM0 = np.pi - ωM * t_out

# 6) PLANET POSITIONS
thetaE = ωE * t
xE = r1 * np.cos(thetaE)
yE = r1 * np.sin(thetaE)
zE = np.zeros_like(t)

thetaM = phiM0 + ωM * t
# give Mars a slight inclination for a 3D effect
i_mars = np.deg2rad(1.85)
xM = r2 * np.cos(thetaM)
yM = r2 * np.sin(thetaM) * np.cos(i_mars)
zM = r2 * np.sin(thetaM) * np.sin(i_mars)

# 7) SPACECRAFT TRAJECTORY
xs = np.zeros_like(t)
ys = np.zeros_like(t)
zs = np.zeros_like(t)

# Phase A: Earth→Mars along the transfer ellipse
for i, ti in enumerate(t1):
    f = ti / t_out
    th = f * np.pi  # true anomaly 0→π
    rr = a * (1 - e**2) / (1 + e * np.cos(th))
    xs[i] = rr * np.cos(th)
    ys[i] = rr * np.sin(th)
    zs[i] = 0

# Phase B: waiting on Mars
for j in range(N2):
    idx = N1 + j
    xs[idx] = xM[idx]
    ys[idx] = yM[idx]
    zs[idx] = zM[idx]

# Phase C: Mars→Earth (return ellipse)
# we must rotate the return ellipse so it departs from actual Mars pos
phi_dep = phiM0 + ωM * (t_out + t_wait)
rot = phi_dep - np.pi  # rotation angle to align ellipse
for k, tk in enumerate(t3):
    idx = N1 + N2 + k
    f = tk / t_back
    th = np.pi + f * np.pi  # true anomaly π→2π
    rr = a * (1 - e**2) / (1 + e * np.cos(th))
    x0 = rr * np.cos(th)
    y0 = rr * np.sin(th)
    # rotate into inertial frame
    xs[idx] = x0 * np.cos(rot) - y0 * np.sin(rot)
    ys[idx] = x0 * np.sin(rot) + y0 * np.cos(rot)
    zs[idx] = 0

# 8) SET UP MATPLOTLIB 3D FIGURE & STATIC ORBITS
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection="3d")
lim = 1.6
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-0.3, 0.3)  # keep z small
ax.set_xlabel("X [AU]")
ax.set_ylabel("Y [AU]")
ax.set_zlabel("Z [AU]")
ax.set_title("Earth→Mars→Earth Trajectory Animation")

# static orbit lines
# Earth orbit (circle in ecliptic)
θ = np.linspace(0, 2 * np.pi, 400)
ax.plot(r1 * np.cos(θ), r1 * np.sin(θ), 0 * θ, "b", lw=0.5)
# Mars orbit (tilted)
ax.plot(
    r2 * np.cos(θ),
    r2 * np.sin(θ) * np.cos(i_mars),
    r2 * np.sin(θ) * np.sin(i_mars),
    "r",
    lw=0.5,
)

# scatter objects
(scE,) = ax.plot([], [], [], "o", color="blue", label="Earth")
(scM,) = ax.plot([], [], [], "o", color="red", label="Mars")
(scS,) = ax.plot([], [], [], "o", color="black", label="Spacecraft")
# tail of spacecraft
(tail,) = ax.plot([], [], [], "-", color="gray", lw=1)

ax.legend(loc="upper right")


# 9) ANIMATION FUNCTION
def update(i):
    scE.set_data(xE[i], yE[i])
    scE.set_3d_properties(zE[i])
    scM.set_data(xM[i], yM[i])
    scM.set_3d_properties(zM[i])
    scS.set_data(xs[i], ys[i])
    scS.set_3d_properties(zs[i])
    # draw tail of the last 50 points
    i0 = max(0, i - 50)
    tail.set_data(xs[i0:i], ys[i0:i])
    tail.set_3d_properties(zs[i0:i])
    return scE, scM, scS, tail


# 10) RUN
ani = FuncAnimation(fig, update, frames=len(t), interval=30, blit=True)

plt.show()
