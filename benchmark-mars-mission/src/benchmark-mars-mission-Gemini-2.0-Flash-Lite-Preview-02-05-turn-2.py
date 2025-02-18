import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import datetime as dt

# --- Constants and Orbital Parameters ---

# Gravitational Constant (km^3/s^2)
G = 6.6743e-20

# Masses (kg)
M_earth = 5.972e24
M_mars = 6.417e23

# Semi-major axes (km)
a_earth = 1.496e8  # Roughly Earth-Sun distance
a_mars = 2.279e8

# Orbital periods (s)
T_earth = 3.154e7  # ~365 days
T_mars = 5.935e7  # ~687 days

# Orbital eccentricities (dimensionless)
e_earth = 0.0167
e_mars = 0.0935

# --- Helper Functions ---


def calculate_orbital_position(a, e, t, T):
    """Calculates position (x, y, z) in a heliocentric orbit at a given time.
    Uses Kepler's Equation and iterative solution.
    """
    # Calculate mean anomaly
    M = (2 * np.pi / T) * t

    # Iteratively solve Kepler's Equation (M = E - e*sin(E))
    E = M  # Initial guess for eccentric anomaly
    for _ in range(10):  # Iterate to refine E
        E = M + e * np.sin(E)

    # Calculate true anomaly (angle from perihelion)
    nu = 2 * np.arctan2(np.sqrt(1 + e) * np.sin(E / 2), np.sqrt(1 - e) * np.cos(E / 2))

    # Calculate radius
    r = a * (1 - e * np.cos(E))

    # Calculate x, y, and z positions (assuming orbit is in the xy plane)
    x = r * np.cos(nu)
    y = r * np.sin(nu)
    z = 0  # Simplified: Assumes orbits are coplanar with the xy plane

    return x, y, z


# --- Trajectory Calculation ---

# Start date
start_date = dt.datetime(2024, 1, 1)

# Time parameters (in seconds)
total_duration = T_earth * 3  # Simulate for about 3 Earth years
num_frames = 360  # Number of animation frames
time_points = np.linspace(0, total_duration, num_frames)

# Pre-allocate arrays to store trajectory data
earth_x = np.zeros(num_frames)
earth_y = np.zeros(num_frames)
earth_z = np.zeros(num_frames)

mars_x = np.zeros(num_frames)
mars_y = np.zeros(num_frames)
mars_z = np.zeros(num_frames)

ship_x = np.zeros(num_frames)
ship_y = np.zeros(num_frames)
ship_z = np.zeros(num_frames)


# Calculate Earth and Mars positions over time
for i, t in enumerate(time_points):
    earth_x[i], earth_y[i], earth_z[i] = calculate_orbital_position(
        a_earth, e_earth, t, T_earth
    )
    mars_x[i], mars_y[i], mars_z[i] = calculate_orbital_position(
        a_mars, e_mars, t, T_mars
    )


# --- Hohmann Transfer Calculation (Simplified)---

# Hohmann transfer is an approximation.  Actual transfers involve more complex delta-v calculations.
# This simplified example uses a series of straight line segments.

# Launch from Earth:  Calculate the time for the Mars arrival

# Determine the optimal launch and arrival times for Mars
# This is a *highly* simplified model and should not be used for mission planning.
# Real Hohmann transfers involve precise timing and orbital mechanics.

# Simplification:  Assume launch window is at time 0 (unrealistic!)
launch_time = 0

# Simplified launch location (start at Earth's position)
ship_x[0] = earth_x[0]
ship_y[0] = earth_y[0]
ship_z[0] = earth_z[0]

# Calculate Mars' position at the arrival time (estimate).  This is the next launch window
arrival_time = launch_time + (
    np.pi * np.sqrt((a_earth + a_mars) ** 3 / (8 * G * 1e-30 * M_sun))
)  # VERY rough estimate of transfer time
mars_arrival_index = np.argmin(np.abs(time_points - arrival_time))

# Landing on Mars (Simplified)
# Simulate a straight line trajectory.
# Note: The code for these segments is *very* simplified and doesn't consider orbital mechanics.

# Mars arrival segment.
for i in range(1, mars_arrival_index):
    ship_x[i] = ship_x[0] + (mars_x[mars_arrival_index] - earth_x[0]) * (
        i / mars_arrival_index
    )
    ship_y[i] = ship_y[0] + (mars_y[mars_arrival_index] - earth_y[0]) * (
        i / mars_arrival_index
    )
    ship_z[i] = ship_z[0] + (mars_z[mars_arrival_index] - earth_z[0]) * (
        i / mars_arrival_index
    )


# Back to Earth (next launch window) - Approximate
#  We need to determine the time for a second launch
return_launch_time = arrival_time + T_mars / 2  # Rough approximation

# Earth arrival.
earth_arrival_index = np.argmin(
    np.abs(time_points - return_launch_time + T_earth / 2)
)  # rough time for returning at the next launch window
# Earth return segment
for i in range(mars_arrival_index, earth_arrival_index):
    ship_x[i] = mars_x[mars_arrival_index] + (
        earth_x[earth_arrival_index] - mars_x[mars_arrival_index]
    ) * ((i - mars_arrival_index) / (earth_arrival_index - mars_arrival_index))
    ship_y[i] = mars_y[mars_arrival_index] + (
        earth_y[earth_arrival_index] - mars_y[mars_arrival_index]
    ) * ((i - mars_arrival_index) / (earth_arrival_index - mars_arrival_index))
    ship_z[i] = mars_z[mars_arrival_index] + (
        earth_z[earth_arrival_index] - mars_z[mars_arrival_index]
    ) * ((i - mars_arrival_index) / (earth_arrival_index - mars_arrival_index))
# --- Animation Setup ---

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection="3d")

# Set axis limits
max_distance = (
    max(
        np.max(np.abs(earth_x)),
        np.max(np.abs(earth_y)),
        np.max(np.abs(mars_x)),
        np.max(np.abs(mars_y)),
    )
    * 1.2
)  # Add some padding
ax.set_xlim([-max_distance, max_distance])
ax.set_ylim([-max_distance, max_distance])
ax.set_zlim([-max_distance, max_distance])
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")
ax.set_title("Earth-Mars-Earth Trajectory (Simplified)")

# Plot the Sun
ax.scatter(0, 0, 0, color="yellow", s=100, label="Sun")

# Plot the orbits as lines
(earth_orbit,) = ax.plot(
    earth_x, earth_y, earth_z, "g-", alpha=0.3, label="Earth Orbit"
)
(mars_orbit,) = ax.plot(mars_x, mars_y, mars_z, "r-", alpha=0.3, label="Mars Orbit")

# Plot the initial positions as points
(earth_point,) = ax.plot(
    [earth_x[0]], [earth_y[0]], [earth_z[0]], "g.", markersize=10, label="Earth"
)
(mars_point,) = ax.plot(
    [mars_x[0]], [mars_y[0]], [mars_z[0]], "r.", markersize=10, label="Mars"
)

# Plot the spacecraft trajectory
(ship_line,) = ax.plot(
    ship_x, ship_y, ship_z, "b-", alpha=0.5, label="Spacecraft Trajectory"
)  # Spacecraft trajectory

# Plot the spacecraft
(ship_point,) = ax.plot(
    [ship_x[0]], [ship_y[0]], [ship_z[0]], "bo", markersize=5, label="Spacecraft"
)  # Spacecraft

ax.legend()


# Animation function
def update(frame):
    # Update planet positions
    earth_point.set_data([earth_x[frame]], [earth_y[frame]])
    earth_point.set_3d_properties([earth_z[frame]])

    mars_point.set_data([mars_x[frame]], [mars_y[frame]])
    mars_point.set_3d_properties([mars_z[frame]])

    # Update spacecraft position
    ship_point.set_data([ship_x[frame]], [ship_y[frame]])
    ship_point.set_3d_properties([ship_z[frame]])

    return (
        earth_point,
        mars_point,
        ship_point,
    )


# Create animation
ani = animation.FuncAnimation(
    fig, update, frames=num_frames, blit=False, repeat=True
)  # Set repeat to True to loop


# Show the plot
plt.show()
