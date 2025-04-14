import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Constants
AU = 1.496e8  # Astronomical Unit in km
earth_orbit_radius = 1  # in AU
mars_orbit_radius = 1.524  # in AU
num_points = 200  # number of points for the orbits
frames_per_phase = 300  # frames for each phase

# Time parameters
total_frames = frames_per_phase * 3  # Earth orbit, transfer, Mars orbit plus return

# Generate Earth and Mars orbits (circular)
theta = np.linspace(0, 2 * np.pi, num_points)
earth_x = earth_orbit_radius * np.cos(theta)
earth_y = earth_orbit_radius * np.sin(theta)

mars_x = mars_orbit_radius * np.cos(theta)
mars_y = mars_orbit_radius * np.sin(theta)


# Function to compute position of Earth and Mars at a given fraction of their orbit
def get_planet_position(orbit_radius, phase_fraction):
    idx = int(phase_fraction * num_points) % num_points
    return orbit_radius * np.cos(theta[idx]), orbit_radius * np.sin(theta[idx])


# Initialize plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-0.5, 0.5)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Launch from Earth to Mars and Back")

# Plot Earth's and Mars's orbits
ax.plot(
    earth_x,
    earth_y,
    np.zeros_like(earth_x),
    color="blue",
    linestyle="--",
    label="Earth Orbit",
)
ax.plot(
    mars_x,
    mars_y,
    np.zeros_like(mars_x),
    color="red",
    linestyle="--",
    label="Mars Orbit",
)

# Initialize objects: planets and spacecraft
(earth_dot,) = ax.plot([], [], [], "bo", label="Earth")
(mars_dot,) = ax.plot([], [], [], "ro", label="Mars")
(spacecraft_line,) = ax.plot([], [], [], "k-", linewidth=2, label="Spacecraft")
(spacecraft_point,) = ax.plot([], [], [], "ko")

# Legend
ax.legend()


# Animation update function
def update(frame):
    # Determine phase based on frame
    if frame < frames_per_phase:
        # Earth orbit
        phase_fraction = frame / frames_per_phase
        ex, ey = get_planet_position(earth_orbit_radius, phase_fraction)
        mx, my = get_planet_position(
            mars_orbit_radius, 0
        )  # Mars stationary during Earth's orbit
        spacecraft_pos = [ex, ey, 0]
        target_pos = [mx, my, 0]
    elif frame < 2 * frames_per_phase:
        # Transfer from Earth to Mars
        t = (frame - frames_per_phase) / frames_per_phase
        ex, ey = get_planet_position(
            earth_orbit_radius, 1
        )  # Earth at full orbit position
        mx, my = get_planet_position(mars_orbit_radius, 0)  # Mars fixed for simplicity

        # Spacecraft moves in a straight line from Earth to Mars
        scx = ex + t * (mx - ex)
        scy = ey + t * (my - ey)
        spacecraft_pos = [scx, scy, 0]

        # For visualization, planets stay stationary during transfer
        ex, ey = get_planet_position(earth_orbit_radius, 1)
        mx, my = get_planet_position(mars_orbit_radius, 0)
    elif frame < 3 * frames_per_phase:
        # Mars orbit
        t_phase = (frame - 2 * frames_per_phase) / frames_per_phase
        mx, my = get_planet_position(mars_orbit_radius, t_phase)
        ex, ey = get_planet_position(earth_orbit_radius, 1)  # Earth fixed
        spacecraft_pos = [mx, my, 0]
    else:
        # Return to Earth at next launch window
        t = (frame - 3 * frames_per_phase) / frames_per_phase
        mx, my = get_planet_position(mars_orbit_radius, 1)  # Mars fixed
        ex, ey = get_planet_position(earth_orbit_radius, 0)  # Earth at start position
        # Spacecraft moves back to Earth
        scx = mx + t * (ex - mx)
        scy = my + t * (ey - my)
        spacecraft_pos = [scx, scy, 0]
        # Update Earth position at start
        ex, ey = get_planet_position(earth_orbit_radius, 0)
        mx, my = get_planet_position(mars_orbit_radius, 1)

    # Update planet positions
    earth_dot.set_data(ex, ey)
    earth_dot.set_3d_properties(0)
    mars_dot.set_data(mx, my)
    mars_dot.set_3d_properties(0)

    # Update spacecraft position
    spacecraft_line.set_data([ex, spacecraft_pos[0]], [ey, spacecraft_pos[1]])
    spacecraft_line.set_3d_properties([0, spacecraft_pos[2]])

    spacecraft_point.set_data([spacecraft_pos[0]], [spacecraft_pos[1]])
    spacecraft_point.set_3d_properties([spacecraft_pos[2]])

    return earth_dot, mars_dot, spacecraft_line, spacecraft_point


# Create animation
ani = FuncAnimation(
    fig, update, frames=total_frames, interval=30, blit=True, repeat=False
)

plt.show()
