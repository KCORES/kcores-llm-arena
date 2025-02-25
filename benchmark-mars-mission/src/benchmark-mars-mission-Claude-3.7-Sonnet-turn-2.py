import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.colors as mcolors

# Set up the figure and 3D axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Constants
AU = 149.6e6  # Astronomical Unit in km
EARTH_RADIUS = 6371  # km
MARS_RADIUS = 3389.5  # km
SUN_RADIUS = 695700  # km (scaled down for visualization)

# Orbital parameters (simplified)
EARTH_ORBIT_RADIUS = 1.0 * AU
MARS_ORBIT_RADIUS = 1.524 * AU
EARTH_YEAR = 365.25  # days
MARS_YEAR = 687  # days


# Create celestial bodies
def create_sphere(radius, color, resolution=20):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    return x, y, z


# Scale for visualization
scale_factor = 0.1
earth_radius_scaled = EARTH_RADIUS * scale_factor
mars_radius_scaled = MARS_RADIUS * scale_factor
sun_radius_scaled = SUN_RADIUS * 0.01 * scale_factor  # Further scaled down

# Create orbits
theta = np.linspace(0, 2 * np.pi, 100)
earth_orbit_x = EARTH_ORBIT_RADIUS * np.cos(theta)
earth_orbit_y = EARTH_ORBIT_RADIUS * np.sin(theta)
earth_orbit_z = np.zeros_like(theta)

mars_orbit_x = MARS_ORBIT_RADIUS * np.cos(theta)
mars_orbit_y = MARS_ORBIT_RADIUS * np.sin(theta)
mars_orbit_z = np.zeros_like(theta)

# Mission parameters
mission_duration = 1000  # days
frames = 200
time = np.linspace(0, mission_duration, frames)

# Earth and Mars positions over time
earth_angle = 2 * np.pi * time / EARTH_YEAR
mars_angle = 2 * np.pi * time / MARS_YEAR

earth_x = EARTH_ORBIT_RADIUS * np.cos(earth_angle)
earth_y = EARTH_ORBIT_RADIUS * np.sin(earth_angle)
earth_z = np.zeros_like(time)

mars_x = MARS_ORBIT_RADIUS * np.cos(mars_angle)
mars_y = MARS_ORBIT_RADIUS * np.sin(mars_angle)
mars_z = np.zeros_like(time)

# Mission trajectory
# Phase 1: Earth to Mars (Hohmann transfer)
outbound_duration = 259  # days for Hohmann transfer
outbound_frames = int(frames * outbound_duration / mission_duration)
outbound_time = np.linspace(0, outbound_duration, outbound_frames)

# Phase 2: Stay on Mars
stay_duration = 500  # days on Mars
stay_frames = int(frames * stay_duration / mission_duration)

# Phase 3: Mars to Earth (Hohmann transfer)
return_duration = 259  # days for return Hohmann transfer
return_frames = frames - outbound_frames - stay_frames

# Create spacecraft trajectory
spacecraft_x = np.zeros(frames)
spacecraft_y = np.zeros(frames)
spacecraft_z = np.zeros(frames)

# Outbound trajectory (Earth to Mars)
for i in range(outbound_frames):
    t = i / outbound_frames
    # Start at Earth's position
    start_x, start_y = earth_x[0], earth_y[0]
    # End at Mars' position after outbound_duration
    end_x, end_y = mars_x[outbound_frames], mars_y[outbound_frames]

    # Hohmann transfer ellipse (simplified)
    a = (EARTH_ORBIT_RADIUS + MARS_ORBIT_RADIUS) / 2
    b = np.sqrt(EARTH_ORBIT_RADIUS * MARS_ORBIT_RADIUS)

    angle = np.pi * t
    r = a * (1 - 0.2 * np.cos(angle))  # Slightly eccentric orbit

    spacecraft_x[i] = r * np.cos(earth_angle[0] + angle)
    spacecraft_y[i] = r * np.sin(earth_angle[0] + angle)
    spacecraft_z[i] = 0  # Simplified to stay in the orbital plane

# Stay on Mars
for i in range(stay_frames):
    idx = outbound_frames + i
    spacecraft_x[idx] = mars_x[idx]
    spacecraft_y[idx] = mars_y[idx]
    spacecraft_z[idx] = mars_z[idx]

# Return trajectory (Mars to Earth)
for i in range(return_frames):
    t = i / return_frames
    idx = outbound_frames + stay_frames + i

    # Start at Mars' position
    start_x, start_y = (
        mars_x[outbound_frames + stay_frames],
        mars_y[outbound_frames + stay_frames],
    )
    # End at Earth's position at the end
    end_x, end_y = earth_x[-1], earth_y[-1]

    # Hohmann transfer ellipse (simplified)
    a = (EARTH_ORBIT_RADIUS + MARS_ORBIT_RADIUS) / 2
    b = np.sqrt(EARTH_ORBIT_RADIUS * MARS_ORBIT_RADIUS)

    angle = np.pi * t
    r = a * (1 - 0.2 * np.cos(np.pi - angle))  # Slightly eccentric orbit

    spacecraft_x[idx] = r * np.cos(mars_angle[outbound_frames + stay_frames] + angle)
    spacecraft_y[idx] = r * np.sin(mars_angle[outbound_frames + stay_frames] + angle)
    spacecraft_z[idx] = 0  # Simplified to stay in the orbital plane


# Animation function
def update(frame):
    ax.clear()

    # Set axis limits
    limit = 1.7 * AU
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit / 2, limit / 2)

    # Plot the Sun
    sun_x, sun_y, sun_z = create_sphere(sun_radius_scaled, "yellow")
    ax.plot_surface(sun_x, sun_y, sun_z, color="yellow", alpha=1)

    # Plot orbits
    ax.plot(
        earth_orbit_x,
        earth_orbit_y,
        earth_orbit_z,
        "b-",
        alpha=0.3,
        label="Earth Orbit",
    )
    ax.plot(
        mars_orbit_x, mars_orbit_y, mars_orbit_z, "r-", alpha=0.3, label="Mars Orbit"
    )

    # Plot Earth
    earth_sphere_x, earth_sphere_y, earth_sphere_z = create_sphere(
        earth_radius_scaled, "blue"
    )
    ax.plot_surface(
        earth_sphere_x + earth_x[frame],
        earth_sphere_y + earth_y[frame],
        earth_sphere_z + earth_z[frame],
        color="blue",
        alpha=0.8,
    )

    # Plot Mars
    mars_sphere_x, mars_sphere_y, mars_sphere_z = create_sphere(
        mars_radius_scaled, "red"
    )
    ax.plot_surface(
        mars_sphere_x + mars_x[frame],
        mars_sphere_y + mars_y[frame],
        mars_sphere_z + mars_z[frame],
        color="red",
        alpha=0.8,
    )

    # Plot spacecraft
    ax.scatter(
        spacecraft_x[frame],
        spacecraft_y[frame],
        spacecraft_z[frame],
        color="white",
        s=50,
        edgecolor="black",
    )

    # Plot trajectory up to current frame
    ax.plot(
        spacecraft_x[: frame + 1],
        spacecraft_y[: frame + 1],
        spacecraft_z[: frame + 1],
        "k-",
        alpha=0.5,
        linewidth=1,
    )

    # Add mission phase indicator
    if frame < outbound_frames:
        phase = "Earth to Mars Transit"
    elif frame < outbound_frames + stay_frames:
        phase = "Mars Surface Operations"
    else:
        phase = "Mars to Earth Return"

    ax.set_title(
        f"Earth-Mars Mission Simulation\nDay {int(time[frame])}: {phase}", fontsize=12
    )

    # Labels and legend
    ax.set_xlabel("X (km)")
    ax.set_ylabel("Y (km)")
    ax.set_zlabel("Z (km)")
    ax.legend(loc="upper right")

    # Improve 3D perspective
    ax.view_init(elev=30, azim=frame / 2)  # Slowly rotate view


# Create animation
ani = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)

plt.tight_layout()
plt.show()

# Uncomment to save animation
# ani.save('earth_mars_mission.mp4', writer='ffmpeg', fps=30, dpi=300)
