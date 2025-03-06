import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
T_e = 365.0  # Earth's orbital period (days)
T_m = 687.0  # Mars' orbital period (days)
r_e = 1.0  # Earth's orbital radius (AU)
r_m = 1.5  # Mars' orbital radius (AU)
a = (r_e + r_m) / 2  # Hohmann transfer semi-major axis (AU)
e = (r_m - r_e) / (r_e + r_m)  # Eccentricity of transfer orbit
T_transfer = T_e * (a / r_e) ** 1.5  # Transfer orbit period (days)

arrival_mars_time = T_transfer / 2.0
departure_mars_time = 780 - T_transfer / 2.0  # Next launch window
end_time = 780.0  # Total mission duration (days)


def solve_kepler(M, e):
    """Newton-Raphson solver for Kepler's equation"""
    E = M  # Initial guess
    for _ in range(100):
        f = E - e * np.sin(E) - M
        f_prime = 1 - e * np.cos(E)
        delta_E = f / f_prime
        E -= delta_E
        if abs(delta_E) < 1e-8:
            break
    return E


def calculate_position(t):
    if t <= arrival_mars_time:
        # Outbound transfer
        period = T_transfer
        mean_anomaly = 2 * np.pi * t / period
        E = solve_kepler(mean_anomaly, e)
    elif departure_mars_time <= t <= end_time:
        # Return transfer
        time_since_departure = t - departure_mars_time
        mean_anomaly = np.pi + (2 * np.pi / T_transfer) * time_since_departure
        E = solve_kepler(mean_anomaly, e)
    else:
        # Landed on Mars
        angle = 2 * np.pi * t / T_m
        return (r_m * np.cos(angle), r_m * np.sin(angle), 0.0)

    cos_E = np.cos(E)
    sin_E = np.sin(E)
    cos_f = (cos_E - e) / (1 - e * cos_E)
    sin_f = (np.sqrt(1 - e**2) * sin_E) / (1 - e * cos_E)
    f = np.arctan2(sin_f, cos_f)
    r = a * (1 - e**2) / (1 + e * cos_E)

    return (r * np.cos(f), r * np.sin(f), 0.0)


# Animation setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot orbits
theta = np.linspace(0, 2 * np.pi, 100)
ax.plot(
    r_e * np.cos(theta),
    r_e * np.sin(theta),
    np.zeros_like(theta),
    "b",
    label="Earth orbit",
)
ax.plot(
    r_m * np.cos(theta),
    r_m * np.sin(theta),
    np.zeros_like(theta),
    "r",
    label="Mars orbit",
)
ax.scatter([0], [0], [0], c="yellow", s=100, label="Sun")

# Initialize objects
earth = ax.plot([], [], [], "bo", markersize=8, label="Earth")[0]
mars = ax.plot([], [], [], "ro", markersize=6, label="Mars")[0]
spacecraft = ax.plot([], [], [], "go", markersize=4, label="Spacecraft")[0]
trajectory = ax.plot([], [], [], "g-", lw=1, label="Trajectory")[0]

ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.legend()


def init():
    earth.set_data([], [])
    earth.set_3d_properties([])
    mars.set_data([], [])
    mars.set_3d_properties([])
    spacecraft.set_data([], [])
    spacecraft.set_3d_properties([])
    trajectory.set_data([], [])
    trajectory.set_3d_properties([])
    return earth, mars, spacecraft, trajectory


def update(frame):
    t = frame

    # Earth's position
    angle_e = 2 * np.pi * t / T_e
    e_x, e_y = r_e * np.cos(angle_e), r_e * np.sin(angle_e)

    # Mars' position
    angle_m = 2 * np.pi * t / T_m
    m_x, m_y = r_m * np.cos(angle_m), r_m * np.sin(angle_m)

    # Spacecraft position
    if arrival_mars_time < t < departure_mars_time:
        s_x, s_y, s_z = m_x, m_y, 0.0
    else:
        s_x, s_y, s_z = calculate_position(t)

    # Update positions
    earth.set_data(e_x, e_y)
    earth.set_3d_properties(r_e * np.sin(angle_e))
    mars.set_data(m_x, m_y)
    mars.set_3d_properties(r_m * np.sin(angle_m))
    spacecraft.set_data(s_x, s_y)
    spacecraft.set_3d_properties(s_z)

    # Update trajectory
    if t == 0:
        trajectory.set_data([], [])
        trajectory.set_3d_properties([])
    else:
        x = np.append(trajectory.get_xdata(), s_x)
        y = np.append(trajectory.get_ydata(), s_y)
        z = np.append(trajectory.get_3d_properties(), s_z)
        trajectory.set_data(x, y)
        trajectory.set_3d_properties(z)

    return earth, mars, spacecraft, trajectory


ani = FuncAnimation(
    fig,
    update,
    frames=np.arange(0, end_time + 1, 1),
    init_func=init,
    blit=True,
    interval=20,
)
plt.show()
