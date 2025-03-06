import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Orbital parameters
T_e = 1.0  # Earth's orbital period (years)
T_m = 1.88  # Mars' orbital period (years)
r_e = 1.0  # Earth's orbital radius (AU)
r_m = 1.5  # Mars' orbital radius (AU)

# Hohmann transfer calculations
a_transfer = (r_e + r_m) / 2
T_transfer = np.sqrt(a_transfer**3)  # Transfer period (years)
T1 = T_transfer / 2  # Time to Mars (years)

# Synodic period (launch window alignment)
synodic_period = 1.0 / (1.0 / T_e - 1.0 / T_m)
T2 = synodic_period - T1  # Waiting time on Mars (years)
T3 = T1  # Return trip time (years)
total_time = T1 + T2 + T3  # Total mission duration

# Precompute critical positions
theta_m_T1 = 2 * np.pi * T1 / T_m
x_m_arrival = r_m * np.cos(theta_m_T1)
y_m_arrival = r_m * np.sin(theta_m_T1)

theta_e_end = 2 * np.pi * total_time / T_e
x_e_end = r_e * np.cos(theta_e_end)
y_e_end = r_e * np.sin(theta_e_end)

theta_m_depart = 2 * np.pi * (T1 + T2) / T_m
x_m_depart = r_m * np.cos(theta_m_depart)
y_m_depart = r_m * np.sin(theta_m_depart)

# Set up the figure and 3D axes
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.1, 0.1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")

# Plot elements
(earth,) = ax.plot([], [], [], "bo", markersize=8, label="Earth")
(mars,) = ax.plot([], [], [], "ro", markersize=6, label="Mars")
(spacecraft,) = ax.plot([], [], [], "go", markersize=4, label="Spacecraft")
(path,) = ax.plot([], [], [], "g-", lw=0.5, alpha=0.5)

spacecraft_path = []


def init():
    path.set_data([], [])
    path.set_3d_properties([])
    return earth, mars, spacecraft, path


def animate(t):
    # Earth's position
    angle_e = 2 * np.pi * t / T_e
    xe = r_e * np.cos(angle_e)
    ye = r_e * np.sin(angle_e)
    ze = 0

    # Mars's position
    angle_m = 2 * np.pi * t / T_m
    xm = r_m * np.cos(angle_m)
    ym = r_m * np.sin(angle_m)
    zm = 0

    # Spacecraft's trajectory
    if t <= T1:
        # To Mars
        x_sc = r_e + (x_m_arrival - r_e) * (t / T1)
        y_sc = (y_m_arrival) * (t / T1)
    elif t <= T1 + T2:
        # Waiting on Mars
        x_sc, y_sc = x_m_arrival, y_m_arrival
    else:
        # Returning to Earth
        delta_t = t - (T1 + T2)
        x_sc = x_m_depart + (x_e_end - x_m_depart) * (delta_t / T3)
        y_sc = y_m_depart + (y_e_end - y_m_depart) * (delta_t / T3)
    z_sc = 0

    # Update spacecraft path
    spacecraft_path.append([x_sc, y_sc, z_sc])
    xs = [p[0] for p in spacecraft_path]
    ys = [p[1] for p in spacecraft_path]
    zs = [p[2] for p in spacecraft_path]
    path.set_data(xs, ys)
    path.set_3d_properties(zs)

    # Update objects
    earth.set_data([xe], [ye])
    earth.set_3d_properties([ze])
    mars.set_data([xm], [ym])
    mars.set_3d_properties([zm])
    spacecraft.set_data([x_sc], [y_sc])
    spacecraft.set_3d_properties([z_sc])

    return earth, mars, spacecraft, path


# Create animation
frames = np.linspace(0, total_time, 200)
ani = FuncAnimation(fig, animate, init_func=init, frames=frames, blit=True, interval=50)
plt.show()
