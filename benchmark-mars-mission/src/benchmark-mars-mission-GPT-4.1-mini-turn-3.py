import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants (simplified and normalized units)
# AU = astronomical unit ~ distance from Earth to Sun
AU = 1  # Define Earth's orbit radius as 1 unit
mars_orbit_radius = 1.524 * AU  # Mars orbit radius in AU

# Orbital periods in Earth years (simplified)
earth_period = 1.0
mars_period = 1.88

# Time parameters (years)
transfer_time_earth_to_mars = 0.5  # approx half the period of Hohmann transfer
stay_duration = 0.5  # stay time on Mars (simplified)
transfer_time_mars_to_earth = 0.5  # transfer time back to Earth

# Number of frames in animation
frames_per_phase = 100

# Angles for Earth orbit (full circle, 0 to 2*pi)
theta_earth = np.linspace(0, 2 * np.pi, frames_per_phase * 4)


# Function to compute planet position in orbit (circular orbit)
def planet_position(radius, period, time):
    # Angular velocity (radians per year)
    omega = 2 * np.pi / period
    angle = omega * time
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0
    return np.array([x, y, z])


# Function to compute spacecraft position during Hohmann transfer (elliptical approx)
def hohmann_transfer(r1, r2, phase):
    """
    phase: 0 to 1, normalized progress along transfer orbit from r1 orbit to r2 orbit

    Returns position in 3D space
    """
    # Semi major axis
    a = (r1 + r2) / 2
    # Eccentricity
    e = (r2 - r1) / (r1 + r2)
    # True anomaly angle corresponding to phase (theta = 180 * phase degrees from perihelion)
    # We'll parametrize the elliptical orbit from
    # perihelion at theta = 0 to aphelion at theta = pi
    theta = np.pi * phase  # from 0 to pi for outward trip

    # Radius at theta
    r = a * (1 - e**2) / (1 + e * np.cos(theta))

    # Position in 2D orbital plane (starting at x=r1 on x-axis, go to r2 at aphelion)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = 0

    return np.array([x, y, z])


# Setup figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.5, 0.5)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Launch from Earth to Mars and back - simplified orbit animation")

# Plot Sun
(sun,) = ax.plot([0], [0], [0], "yo", markersize=12, label="Sun")

# Plot Earth and Mars orbits
(earth_orbit_line,) = ax.plot([], [], [], "b--", label="Earth Orbit")
(mars_orbit_line,) = ax.plot([], [], [], "r--", label="Mars Orbit")

# Plot planets positions
(earth_pos_plot,) = ax.plot([], [], [], "bo", markersize=8, label="Earth")
(mars_pos_plot,) = ax.plot([], [], [], "ro", markersize=8, label="Mars")

# Spacecraft trajectory
(sc_plot,) = ax.plot([], [], [], "g-o", markersize=6, label="Spacecraft")

# Legend
ax.legend()


# Pre-compute the circular orbits for orbit lines
theta_orbit = np.linspace(0, 2 * np.pi, 300)
earth_orbit_x = AU * np.cos(theta_orbit)
earth_orbit_y = AU * np.sin(theta_orbit)
earth_orbit_z = np.zeros_like(theta_orbit)

mars_orbit_x = mars_orbit_radius * np.cos(theta_orbit)
mars_orbit_y = mars_orbit_radius * np.sin(theta_orbit)
mars_orbit_z = np.zeros_like(theta_orbit)

earth_orbit_line.set_data(earth_orbit_x, earth_orbit_y)
earth_orbit_line.set_3d_properties(earth_orbit_z)

mars_orbit_line.set_data(mars_orbit_x, mars_orbit_y)
mars_orbit_line.set_3d_properties(mars_orbit_z)

# Total frames calculation
total_frames = 4 * frames_per_phase  # earth orbit, transfer, stay, transfer back


def update(frame):
    ax.view_init(elev=30, azim=frame * 0.5)  # rotate view slowly

    if frame < frames_per_phase:
        # Phase 1: Earth orbit before launch
        t = frame / frames_per_phase * earth_period
        earth_pos = planet_position(AU, earth_period, t)
        mars_pos = planet_position(mars_orbit_radius, mars_period, t)
        sc_pos = earth_pos  # spacecraft on Earth

    elif frame < 2 * frames_per_phase:
        # Phase 2: Transfer Earth -> Mars
        t = frames_per_phase / frames_per_phase * earth_period  # launch time fixed
        transfer_phase = (frame - frames_per_phase) / frames_per_phase
        # Positions of planets at start and end of transfer
        earth_pos_start = planet_position(AU, earth_period, t)
        mars_pos_end = planet_position(
            mars_orbit_radius, mars_period, t + transfer_time_earth_to_mars
        )

        # Spacecraft position along Hohmann transfer orbit rotated to match actual planet positions
        # For simplicity, transfer orbit lies in the plane of Earth position to Mars position
        # Compute angle difference between Earth's and Mars' position
        start_vec = earth_pos_start[:2]
        end_vec = mars_pos_end[:2]
        angle_between = np.arctan2(end_vec[1], end_vec[0]) - np.arctan2(
            start_vec[1], start_vec[0]
        )

        # Compute spacecraft position on transfer orbit
        s_pos = hohmann_transfer(AU, mars_orbit_radius, transfer_phase)

        # Rotate s_pos by start angle so transfer orbit is aligned with Earth position
        c, s = np.cos(np.arctan2(start_vec[1], start_vec[0])), np.sin(
            np.arctan2(start_vec[1], start_vec[0])
        )
        rot_mat = np.array([[c, -s], [s, c]])
        s_pos_2d = rot_mat @ s_pos[:2]
        sc_pos = np.array([s_pos_2d[0], s_pos_2d[1], 0])

        earth_pos = earth_pos_start
        mars_pos = mars_pos_end

    elif frame < 3 * frames_per_phase:
        # Phase 3: Stay on Mars
        t = (
            frames_per_phase
            + transfer_time_earth_to_mars
            + (frame - 2 * frames_per_phase) / frames_per_phase * stay_duration
        )
        earth_pos = planet_position(AU, earth_period, t)
        mars_pos = planet_position(mars_orbit_radius, mars_period, t)
        sc_pos = mars_pos  # spacecraft landed on Mars

    else:
        # Phase 4: Transfer Mars -> Earth
        t = frames_per_phase + transfer_time_earth_to_mars + stay_duration
        transfer_phase = (frame - 3 * frames_per_phase) / frames_per_phase
        # Positions of planets at start and end of transfer
        mars_pos_start = planet_position(mars_orbit_radius, mars_period, t)
        earth_pos_end = planet_position(
            AU, earth_period, t + transfer_time_mars_to_earth
        )

        # Spacecraft position along return Hohmann transfer orbit (elliptical)
        s_pos = hohmann_transfer(mars_orbit_radius, AU, transfer_phase)

        # Rotate transfer orbit to start at Mars position
        start_vec = mars_pos_start[:2]
        c, s = np.cos(np.arctan2(start_vec[1], start_vec[0])), np.sin(
            np.arctan2(start_vec[1], start_vec[0])
        )
        rot_mat = np.array([[c, -s], [s, c]])
        s_pos_2d = rot_mat @ s_pos[:2]
        sc_pos = np.array([s_pos_2d[0], s_pos_2d[1], 0])

        earth_pos = earth_pos_end
        mars_pos = mars_pos_start

    # Update planet positions
    earth_pos_plot.set_data(earth_pos[0], earth_pos[1])
    earth_pos_plot.set_3d_properties(earth_pos[2])

    mars_pos_plot.set_data(mars_pos[0], mars_pos[1])
    mars_pos_plot.set_3d_properties(mars_pos[2])

    # Update spacecraft position
    sc_plot.set_data(sc_pos[0], sc_pos[1])
    sc_plot.set_3d_properties(sc_pos[2])

    return earth_pos_plot, mars_pos_plot, sc_plot


ani = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=True)

plt.show()
