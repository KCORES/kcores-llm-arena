import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e8  # km, 1 astronomical unit
earth_orbit_radius = 1.0 * AU
mars_orbit_radius = 1.524 * AU
earth_period = 365  # days
mars_period = 687  # days
transfer_time = 259  # days
waiting_time = 521  # days
total_time = transfer_time + waiting_time + transfer_time  # 1039 days

# Time steps for animation
time_step = 1  # days
times = np.arange(0, total_time + time_step, time_step)

# Calculate planet positions
earth_angles = (times * 360 / earth_period) % 360
mars_angles = (44 + times * 360 / mars_period) % 360

earth_x = earth_orbit_radius * np.cos(np.radians(earth_angles))
earth_y = earth_orbit_radius * np.sin(np.radians(earth_angles))
earth_z = np.zeros_like(earth_x)

mars_x = mars_orbit_radius * np.cos(np.radians(mars_angles))
mars_y = mars_orbit_radius * np.sin(np.radians(mars_angles))
mars_z = np.zeros_like(mars_x)

# Convert to AU for plotting
earth_x_au = earth_x / AU
earth_y_au = earth_y / AU
earth_z_au = earth_z / AU

mars_x_au = mars_x / AU
mars_y_au = mars_y / AU
mars_z_au = mars_z / AU


# Kepler equation solver using Newton-Raphson
def kepler_equation(M, e, tol=1e-8, max_iter=100):
    E = M  # Initial guess
    for _ in range(max_iter):
        f = E - e * np.sin(E) - M
        f_prime = 1 - e * np.cos(E)
        E_new = E - f / f_prime
        if abs(E_new - E) < tol:
            return E_new
        E = E_new
    return E


# Spacecraft trajectory calculation
a_transfer = (earth_orbit_radius + mars_orbit_radius) / 2
e_transfer = (mars_orbit_radius - earth_orbit_radius) / (
    mars_orbit_radius + earth_orbit_radius
)

sc_x = []
sc_y = []
sc_z = []

for t in times:
    if t <= transfer_time:
        # Outbound transfer to Mars
        M = (2 * np.pi / transfer_time) * t
        E = kepler_equation(M, e_transfer)
        nu = 2 * np.arctan(np.sqrt((1 + e_transfer) / (1 - e_transfer)) * np.tan(E / 2))
        r = a_transfer * (1 - e_transfer * np.cos(E))
        x = r * np.cos(nu)
        y = r * np.sin(nu)
        z = 0
    elif t <= transfer_time + waiting_time:
        # Stay on Mars
        mars_angle = mars_angles[int(t)]
        x = mars_orbit_radius * np.cos(np.radians(mars_angle))
        y = mars_orbit_radius * np.sin(np.radians(mars_angle))
        z = 0
    else:
        # Return transfer to Earth
        t_return = t - (transfer_time + waiting_time)
        M_return = (2 * np.pi / transfer_time) * t_return + np.pi
        E_return = kepler_equation(M_return, e_transfer)
        nu_return = 2 * np.arctan(
            np.sqrt((1 + e_transfer) / (1 - e_transfer)) * np.tan(E_return / 2)
        )
        r_return = a_transfer * (1 - e_transfer * np.cos(E_return))
        x = r_return * np.cos(nu_return)
        y = r_return * np.sin(nu_return)
        z = 0
    sc_x.append(x / AU)
    sc_y.append(y / AU)
    sc_z.append(z / AU)

# Set up figure
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.view_init(elev=30, azim=45)

# Plot orbits
ax.plot(
    earth_x_au, earth_y_au, earth_z_au, color="blue", alpha=0.3, label="Earth Orbit"
)
ax.plot(mars_x_au, mars_y_au, mars_z_au, color="red", alpha=0.3, label="Mars Orbit")

# Initialize animation elements
(earth_line,) = ax.plot([], [], [], "o", color="blue", label="Earth")
(mars_line,) = ax.plot([], [], [], "o", color="red", label="Mars")
(sc_line,) = ax.plot([], [], [], "o", color="green", label="Spacecraft")
(trajectory_line,) = ax.plot([], [], [], color="green", alpha=0.3)


def init():
    earth_line.set_data([], [])
    earth_line.set_3d_properties([])
    mars_line.set_data([], [])
    mars_line.set_3d_properties([])
    sc_line.set_data([], [])
    sc_line.set_3d_properties([])
    trajectory_line.set_data([], [])
    trajectory_line.set_3d_properties([])
    return earth_line, mars_line, sc_line, trajectory_line


def animate(i):
    # Update planet positions
    earth_line.set_data([earth_x_au[i]], [earth_y_au[i]])
    earth_line.set_3d_properties([earth_z_au[i]])

    mars_line.set_data([mars_x_au[i]], [mars_y_au[i]])
    mars_line.set_3d_properties([mars_z_au[i]])

    # Update spacecraft position
    sc_line.set_data([sc_x[i]], [sc_y[i]])
    sc_line.set_3d_properties([sc_z[i]])

    # Update trajectory
    if i > 0:
        trajectory_line.set_data(sc_x[: i + 1], sc_y[: i + 1])
        trajectory_line.set_3d_properties(sc_z[: i + 1])
    else:
        trajectory_line.set_data([], [])
        trajectory_line.set_3d_properties([])

    return earth_line, mars_line, sc_line, trajectory_line


ani = FuncAnimation(
    fig, animate, frames=len(times), init_func=init, blit=True, interval=50
)
plt.legend()
plt.show()
