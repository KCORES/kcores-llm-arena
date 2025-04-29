import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Constants
AU = 1.0
r_Earth = 1.0
r_Mars = 1.524
T_Earth = 1.0  # Year
T_Mars = 1.88  # Year
omega_Earth = 2 * np.pi / T_Earth
omega_Mars = 2 * np.pi / T_Mars
phi_Mars_0 = 0.7676  # Initial angle of Mars (radians)

# Mission timeline (in years)
t_transfer_out = 0.71
t_arrive_Mars = t_transfer_out
t_depart_Mars = 1.948
t_transfer_back = 0.71
t_return_Earth = t_depart_Mars + t_transfer_back
total_time = t_return_Earth


# Position functions
def earth_position(t):
    theta = omega_Earth * t
    x = r_Earth * np.cos(theta)
    y = r_Earth * np.sin(theta)
    z = 0.0
    return x, y, z


def mars_position(t):
    theta = omega_Mars * t + phi_Mars_0
    x = r_Mars * np.cos(theta)
    y = r_Mars * np.sin(theta)
    z = 0.0
    return x, y, z


def spacecraft_position(t):
    if t <= t_arrive_Mars:
        s = t / t_arrive_Mars
        x = (1 - s) ** 2 * r_Earth + s**2 * (-r_Mars)
        y = 2 * (1 - s) * s * 1.5
        z = 0.0
    elif t <= t_depart_Mars:
        x, y, z = mars_position(t)
    else:
        s = (t - t_depart_Mars) / (t_return_Earth - t_depart_Mars)
        x_start, y_start, _ = mars_position(t_depart_Mars)
        x_end, y_end, _ = earth_position(t_return_Earth)
        x = (1 - s) ** 2 * x_start + s**2 * x_end
        y = (1 - s) ** 2 * y_start + 2 * (1 - s) * s * (-1.5) + s**2 * y_end
        z = 0.0
    return x, y, z


# Setup figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Axis limits
limit = 2.0
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-0.1, 0.1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Earth to Mars Roundtrip Mission Animation")

# Earth and Mars orbits
theta = np.linspace(0, 2 * np.pi, 200)
x_orbit_E = r_Earth * np.cos(theta)
y_orbit_E = r_Earth * np.sin(theta)
z_orbit_E = np.zeros_like(theta)
ax.plot(x_orbit_E, y_orbit_E, z_orbit_E, "g--", alpha=0.3)

x_orbit_M = r_Mars * np.cos(theta)
y_orbit_M = r_Mars * np.sin(theta)
ax.plot(x_orbit_M, y_orbit_M, z_orbit_E, "r--", alpha=0.3)

# Sun
ax.scatter(0, 0, 0, color="orange", s=300, label="Sun")

# Initialize spacecraft and planets
(earth_dot,) = ax.plot([], [], [], "go", markersize=10)
(mars_dot,) = ax.plot([], [], [], "ro", markersize=8)
(sc_path,) = ax.plot(
    [], [], [], "b*-", linewidth=1, markevery=100, label="Spacecraft Path"
)


def init():
    earth_dot.set_data([], [])
    earth_dot.set_3d_properties([])
    mars_dot.set_data([], [])
    mars_dot.set_3d_properties([])
    sc_path.set_data([], [])
    sc_path.set_3d_properties([])
    return earth_dot, mars_dot, sc_path


# Global storage for spacecraft path
sc_x, sc_y, sc_z = [], [], []


def update(t):
    xe, ye, ze = earth_position(t)
    xm, ym, zm = mars_position(t)
    xsc, ysc, zsc = spacecraft_position(t)

    earth_dot.set_data(xe, ye)
    earth_dot.set_3d_properties(ze)

    mars_dot.set_data(xm, ym)
    mars_dot.set_3d_properties(zm)

    sc_x.append(xsc)
    sc_y.append(ysc)
    sc_z.append(zsc)
    sc_path.set_data(sc_x, sc_y)
    sc_path.set_3d_properties(sc_z)

    ax.set_title(f"Time: {t:.2f} Years — Earth to Mars Roundtrip Mission")

    return earth_dot, mars_dot, sc_path


# Frame settings
time_step = 0.02
times = np.arange(0, total_time + time_step, time_step)

ani = FuncAnimation(
    fig, update, frames=times, init_func=init, blit=False, interval=50, repeat=False
)

plt.legend(loc="upper right")
plt.tight_layout()
plt.show()
