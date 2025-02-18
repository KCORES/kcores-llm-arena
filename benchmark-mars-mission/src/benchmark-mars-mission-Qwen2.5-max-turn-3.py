import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Constants
earth_orbit_radius = 1.0  # AU (Astronomical Unit)
mars_orbit_radius = 1.52  # AU
orbital_period_earth = 1.0  # Year
orbital_period_mars = 1.88  # Year
animation_frames = 360  # Number of frames for the animation
travel_time_to_mars = 0.7  # Approximate time in years for Earth-to-Mars transfer
stay_time_on_mars = 0.5  # Time spent on Mars before returning
travel_time_to_earth = 0.7  # Approximate time in years for Mars-to-Earth transfer

# Generate Earth's and Mars' positions over time
theta = np.linspace(0, 2 * np.pi, animation_frames)
earth_x = earth_orbit_radius * np.cos(theta)
earth_y = earth_orbit_radius * np.sin(theta)
earth_z = np.zeros_like(earth_x)

mars_x = mars_orbit_radius * np.cos(theta / orbital_period_mars)
mars_y = mars_orbit_radius * np.sin(theta / orbital_period_mars)
mars_z = np.zeros_like(mars_x)


# Simulate spacecraft trajectory
def spacecraft_trajectory(frame):
    total_time = travel_time_to_mars + stay_time_on_mars + travel_time_to_earth
    normalized_time = frame / animation_frames * total_time

    if normalized_time < travel_time_to_mars:
        # Traveling from Earth to Mars
        t = normalized_time / travel_time_to_mars
        x = (1 - t) * earth_x[frame] + t * mars_x[frame]
        y = (1 - t) * earth_y[frame] + t * mars_y[frame]
        z = (1 - t) * earth_z[frame] + t * mars_z[frame]
    elif normalized_time < travel_time_to_mars + stay_time_on_mars:
        # Staying on Mars
        x = mars_x[frame]
        y = mars_y[frame]
        z = mars_z[frame]
    else:
        # Traveling back to Earth
        t = (
            normalized_time - travel_time_to_mars - stay_time_on_mars
        ) / travel_time_to_earth
        x = (1 - t) * mars_x[frame] + t * earth_x[frame]
        y = (1 - t) * mars_y[frame] + t * earth_y[frame]
        z = (1 - t) * mars_z[frame] + t * earth_z[frame]

    return x, y, z


# Set up the plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(-1, 1)
ax.set_xlabel("X (AU)")
ax.set_ylabel("Y (AU)")
ax.set_zlabel("Z (AU)")
ax.set_title("Spacecraft Trajectory: Earth → Mars → Earth")

# Plot orbits
ax.plot(earth_x, earth_y, earth_z, label="Earth Orbit", color="blue", linestyle="--")
ax.plot(mars_x, mars_y, mars_z, label="Mars Orbit", color="red", linestyle="--")

# Initialize spacecraft and planets
(spacecraft,) = ax.plot([], [], [], marker="o", color="black", label="Spacecraft")
(earth,) = ax.plot([], [], [], marker="o", color="blue", label="Earth")
(mars,) = ax.plot([], [], [], marker="o", color="red", label="Mars")


# Animation update function
def update(frame):
    # Update Earth and Mars positions
    earth.set_data([earth_x[frame]], [earth_y[frame]])
    earth.set_3d_properties([earth_z[frame]])
    mars.set_data([mars_x[frame]], [mars_y[frame]])
    mars.set_3d_properties([mars_z[frame]])

    # Update spacecraft position
    x, y, z = spacecraft_trajectory(frame)
    spacecraft.set_data([x], [y])
    spacecraft.set_3d_properties([z])

    return spacecraft, earth, mars


# Create the animation
ani = FuncAnimation(fig, update, frames=animation_frames, interval=50, blit=False)

# Add legend
ax.legend()

# Show the animation
plt.show()
