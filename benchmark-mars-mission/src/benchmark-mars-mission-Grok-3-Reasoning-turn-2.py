import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
AU = 149.6e6  # km, for scaling (1 AU in km)
t_max = 1209  # Total days, covering launch, stay, and return
fps = 30
frames = t_max * fps // 24  # Approximate frames, 24 hours per day for smooth animation

# Orbital parameters
earth_radius = 1.0 * AU  # Earth's orbit radius in km
mars_radius = 1.524 * AU  # Mars's orbit radius in km
earth_period = 365.25  # Days
mars_period = 687.0  # Days
omega_earth = 2 * np.pi / earth_period  # Rad/day
omega_mars = 2 * np.pi / mars_period  # Rad/day

# Transfer orbit parameters (Hohmann transfer)
a_transfer = (earth_radius + mars_radius) / 2  # Semi-major axis for transfer
T_transfer = np.sqrt(a_transfer**3 / (4 * np.pi**2)) * 365.25  # Period in days, scaled
transfer_time = T_transfer / 2  # Half period for one-way trip, approximately 259 days
return_transfer_time = 250  # Approximate return trip, slightly less

# Initialize figure
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-1.6 * AU, 1.6 * AU)
ax.set_ylim(-1.6 * AU, 1.6 * AU)
ax.set_zlim(-0.1 * AU, 0.1 * AU)
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Animated 3D Plot: Earth to Mars and Back")

# Plot orbits
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit_x = earth_radius * np.cos(theta)
earth_orbit_y = earth_radius * np.sin(theta)
mars_orbit_x = mars_radius * np.cos(theta)
mars_orbit_y = mars_radius * np.sin(theta)
ax.plot(
    earth_orbit_x,
    earth_orbit_y,
    zs=0,
    color="blue",
    label="Earth Orbit",
    linestyle="--",
)
ax.plot(
    mars_orbit_x, mars_orbit_y, zs=0, color="red", label="Mars Orbit", linestyle="--"
)
ax.legend()

# Initialize plot elements
(earth,) = ax.plot([], [], "bo", markersize=10, label="Earth")
(mars,) = ax.plot([], [], "ro", markersize=8, label="Mars")
(spacecraft,) = ax.plot([], [], "go", markersize=5, label="Spacecraft")
(earth_trail,) = ax.plot([], [], "b-", alpha=0.3)
(mars_trail,) = ax.plot([], [], "r-", alpha=0.3)
(spacecraft_trail,) = ax.plot([], [], "g-", alpha=0.3)


def init():
    earth.set_data([], [])
    mars.set_data([], [])
    spacecraft.set_data([], [])
    earth_trail.set_data([], [])
    mars_trail.set_data([], [])
    spacecraft_trail.set_data([], [])
    return earth, mars, spacecraft, earth_trail, mars_trail, spacecraft_trail


def update(frame):
    t = frame / fps * 24  # Time in days, scaled for animation speed

    # Calculate positions
    earth_angle = omega_earth * t
    earth_x = earth_radius * np.cos(earth_angle)
    earth_y = earth_radius * np.sin(earth_angle)

    mars_angle = omega_mars * t
    mars_x = mars_radius * np.cos(mars_angle)
    mars_y = mars_radius * np.sin(mars_angle)

    # Spacecraft position
    if t <= transfer_time:  # Earth to Mars
        # Simplified linear interpolation for Hohmann transfer
        progress = t / transfer_time
        spacecraft_x = (
            earth_x * (1 - progress) + mars_x * progress
        )  # Linear for simplicity
        spacecraft_y = earth_y * (1 - progress) + mars_y * progress
    elif t <= transfer_time + 730:  # Stay on Mars, approximately 2 years
        spacecraft_x = mars_x
        spacecraft_y = mars_y
    elif t <= transfer_time + 730 + return_transfer_time:  # Mars to Earth
        start_return = transfer_time + 730
        progress = (t - start_return) / return_transfer_time
        spacecraft_x = (
            mars_x * (1 - progress) + earth_x * progress
        )  # Linear for simplicity
        spacecraft_y = mars_y * (1 - progress) + earth_y * progress
    else:
        spacecraft_x = earth_x
        spacecraft_y = earth_y

    # Update plot
    earth.set_data([earth_x], [earth_y])
    mars.set_data([mars_x], [mars_y])
    spacecraft.set_data([spacecraft_x], [spacecraft_y])

    # Update trails (last 100 frames for visibility)
    earth_trail_x = np.append(earth_trail.get_xdata(), earth_x)[-100:]
    earth_trail_y = np.append(earth_trail.get_ydata(), earth_y)[-100:]
    mars_trail_x = np.append(mars_trail.get_xdata(), mars_x)[-100:]
    mars_trail_y = np.append(mars_trail.get_ydata(), mars_y)[-100:]
    spacecraft_trail_x = np.append(spacecraft_trail.get_xdata(), spacecraft_x)[-100:]
    spacecraft_trail_y = np.append(spacecraft_trail.get_ydata(), spacecraft_y)[-100:]

    earth_trail.set_data(earth_trail_x, earth_trail_y)
    mars_trail.set_data(mars_trail_x, mars_trail_y)
    spacecraft_trail.set_data(spacecraft_trail_x, spacecraft_trail_y)

    return earth, mars, spacecraft, earth_trail, mars_trail, spacecraft_trail


ani = FuncAnimation(
    fig, update, init_func=init, frames=frames, interval=1000 / fps, blit=True
)
plt.show()
