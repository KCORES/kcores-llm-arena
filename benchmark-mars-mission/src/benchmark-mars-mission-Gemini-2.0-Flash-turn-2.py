import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

# --- Physical Constants and Parameters ---
G = 6.67430e-11  # Gravitational constant
M_earth = 5.972e24  # Mass of Earth
M_mars = 6.417e23  # Mass of Mars
R_earth = 6.371e6  # Radius of Earth
R_mars = 3.3895e6  # Radius of Mars
AU = 1.496e11  # Astronomical Unit

# --- Orbital Parameters (Simplified - Circular Orbits) ---
earth_orbit_radius = AU
mars_orbit_radius = 1.524 * AU  # Semi-major axis

# Assume circular orbits and coplanar orbits for simplicity

earth_orbital_period = 365.25 * 24 * 3600  # Earth's orbital period in seconds
mars_orbital_period = 687 * 24 * 3600  # Mars' orbital period in seconds


# --- Hohmann Transfer Orbit Calculation ---
def hohmann_transfer_velocity(r1, r2, mu):
    """Calculates the velocities required for a Hohmann transfer orbit."""
    a_transfer = (r1 + r2) / 2  # Semi-major axis of the transfer orbit
    v1 = np.sqrt(mu / r1)  # Circular orbital velocity at r1
    v2 = np.sqrt(mu / r2)  # Circular orbital velocity at r2
    v_transfer1 = np.sqrt(
        mu * ((2 / r1) - (1 / a_transfer))
    )  # Velocity at r1 of transfer orbit
    v_transfer2 = np.sqrt(
        mu * ((2 / r2) - (1 / a_transfer))
    )  # Velocity at r2 of transfer orbit
    delta_v1 = v_transfer1 - v1  # Velocity change at Earth
    delta_v2 = v2 - v_transfer2  # Velocity change at Mars
    return delta_v1, delta_v2


# --- Launch Window Calculation ---
def launch_window_angle(earth_orbital_period, mars_orbital_period):
    """Calculates the phase angle for a Hohmann transfer from Earth to Mars."""
    return np.pi - np.pi * (earth_orbital_period / mars_orbital_period)


# --- Equations of Motion ---
def equations_of_motion(t, state, mu):
    """Equations of motion for a single body under gravity."""
    x, y, z, vx, vy, vz = state
    r = np.sqrt(x**2 + y**2 + z**2)
    ax = -mu * x / r**3
    ay = -mu * y / r**3
    az = -mu * z / r**3
    return [vx, vy, vz, ax, ay, az]


# --- Earth and Mars Positions over Time ---
def earth_mars_positions(t):
    """Calculates the position of Earth and Mars at a given time."""
    earth_angle = (2 * np.pi * t) / earth_orbital_period
    mars_angle = (2 * np.pi * t) / mars_orbital_period

    earth_x = earth_orbit_radius * np.cos(earth_angle)
    earth_y = earth_orbit_radius * np.sin(earth_angle)
    earth_z = 0  # Assuming coplanar orbits

    mars_x = mars_orbit_radius * np.cos(mars_angle)
    mars_y = mars_orbit_radius * np.sin(mars_angle)
    mars_z = 0

    return earth_x, earth_y, earth_z, mars_x, mars_y, mars_z


# --- Simulation ---

# 1. Calculate initial conditions for Earth-Mars transfer
delta_v1, delta_v2 = hohmann_transfer_velocity(
    earth_orbit_radius, mars_orbit_radius, G * M_sun
)  # mu is actually M_sun
earth_orbital_velocity = np.sqrt(G * M_sun / earth_orbit_radius)
# Assuming at time 0, the earth is at (earth_orbit_radius, 0, 0) and moving in the +y direction
initial_transfer_vx = 0
initial_transfer_vy = (
    earth_orbital_velocity + delta_v1
)  # Added delta v to earth's velocity

# 2. Solve for the Earth-Mars trajectory
t_span_em = (
    0,
    np.sqrt(((earth_orbit_radius + mars_orbit_radius) / 2) ** 3 / (G * M_sun)) * np.pi,
)  # Half an ellipse in time
initial_state_em = [
    earth_orbit_radius,
    0,
    0,
    initial_transfer_vx,
    initial_transfer_vy,
    0,
]
solution_em = solve_ivp(
    equations_of_motion,
    t_span_em,
    initial_state_em,
    dense_output=True,
    args=(G * M_sun,),
)

# 3. Find arrival window
phase_angle = launch_window_angle(earth_orbital_period, mars_orbital_period)
launch_time = 0  # Start at t=0, we already know that it will launch when phase angle = phase_angle
mars_arrival_angle = phase_angle + (2 * np.pi * t_span_em[1]) / mars_orbital_period
# find the time that mars arrives
mars_launch_time = 0
for time in solution_em.t:
    mars_x, mars_y, mars_z = earth_mars_positions(time)[3:6]
    mars_arrival_angle_actual = np.arctan2(mars_y, mars_x)
    if (
        abs(mars_arrival_angle_actual - mars_arrival_angle) < 0.01
    ):  # Within 0.01 radians (about 0.5 degrees)
        mars_launch_time = time
        break

# 4. Calcluate Mars-Earth transfer
delta_v1_me, delta_v2_me = hohmann_transfer_velocity(
    mars_orbit_radius, earth_orbit_radius, G * M_sun
)  # mu is actually M_sun
mars_orbital_velocity = np.sqrt(G * M_sun / mars_orbit_radius)
# Assuming at time 0, the earth is at (earth_orbit_radius, 0, 0) and moving in the +y direction
initial_transfer_vx_me = 0
initial_transfer_vy_me = -(
    mars_orbital_velocity + delta_v1_me
)  # Added delta v to earth's velocity

# 5. Solve for the Mars-Earth trajectory
t_span_me = (
    0,
    np.sqrt(((earth_orbit_radius + mars_orbit_radius) / 2) ** 3 / (G * M_sun)) * np.pi,
)  # Half an ellipse in time
initial_state_me = [
    mars_orbit_radius,
    0,
    0,
    initial_transfer_vx_me,
    initial_transfer_vy_me,
    0,
]
solution_me = solve_ivp(
    equations_of_motion,
    t_span_me,
    initial_state_me,
    dense_output=True,
    args=(G * M_sun,),
)

# 6 Calculate Return Window
earth_arrival_angle = phase_angle - (2 * np.pi * t_span_me[1]) / earth_orbital_period
# find the time that earth arrives
earth_launch_time = 0
for time in solution_me.t:
    earth_x, earth_y, earth_z = earth_mars_positions(time)[0:3]
    earth_arrival_angle_actual = np.arctan2(earth_y, earth_x)
    if (
        abs(earth_arrival_angle_actual - earth_arrival_angle) < 0.01
    ):  # Within 0.01 radians (about 0.5 degrees)
        earth_launch_time = time
        break

# Combine Earth-Mars and Mars-Earth Trajectories
total_time = mars_launch_time + earth_launch_time  # Total time to go to mars and return
t_eval = np.linspace(0, total_time, 500)  # Evaluate the trajectories at these times

# Arrays to store trajectory points for plotting
trajectory_x = []
trajectory_y = []
trajectory_z = []

for t in t_eval:
    if t <= mars_launch_time:
        solution_eval = solution_em.sol(t)  # evaluate the first trajectory
        trajectory_x.append(solution_eval[0])
        trajectory_y.append(solution_eval[1])
        trajectory_z.append(solution_eval[2])
    else:
        solution_eval = solution_me.sol(
            t - mars_launch_time
        )  # Evaluate the return trajectory, with a time offset of mars_launch_time
        trajectory_x.append(solution_eval[0])
        trajectory_y.append(solution_eval[1])
        trajectory_z.append(solution_eval[2])

trajectory_x = np.array(trajectory_x)
trajectory_y = np.array(trajectory_y)
trajectory_z = np.array(trajectory_z)


# --- 3D Plotting and Animation ---
fig = plt.figure(figsize=(12, 12))
ax = fig.add_subplot(111, projection="3d")

# Planet spheres (simplified)
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 100)

# Earth sphere
earth_x = R_earth * np.outer(np.cos(u), np.sin(v))
earth_y = R_earth * np.outer(np.sin(u), np.sin(v))
earth_z = R_earth * np.outer(np.ones(np.size(u)), np.cos(v))

# Mars sphere
mars_x = R_mars * np.outer(np.cos(u), np.sin(v))
mars_y = R_mars * np.outer(np.sin(u), np.sin(v))
mars_z = R_mars * np.outer(np.ones(np.size(u)), np.cos(v))

# Trajectory line
(line,) = ax.plot([], [], [], lw=2, color="red")

# Earth and Mars markers that move in orbit
(earth_marker,) = ax.plot(
    [earth_orbit_radius], [0], [0], marker="o", markersize=10, color="blue"
)
(mars_marker,) = ax.plot(
    [mars_orbit_radius], [0], [0], marker="o", markersize=8, color="orange"
)


# --- Animation Function ---
def animate(i):
    """Animation function to update the plot in each frame."""

    # Update Earth and Mars positions
    time = t_eval[i]
    earth_x_pos, earth_y_pos, earth_z_pos, mars_x_pos, mars_y_pos, mars_z_pos = (
        earth_mars_positions(time)
    )

    earth_marker.set_data([earth_x_pos], [earth_y_pos])
    earth_marker.set_3d_properties([earth_z_pos])

    mars_marker.set_data([mars_x_pos], [mars_y_pos])
    mars_marker.set_3d_properties([mars_z_pos])

    # Update trajectory line
    line.set_data(trajectory_x[:i], trajectory_y[:i])
    line.set_3d_properties(trajectory_z[:i])

    return line, earth_marker, mars_marker


# --- Plot Setup ---
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-2 * AU, 2 * AU])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Earth-Mars-Earth Transfer Orbit")
ax.view_init(elev=30, azim=45)  # Adjust viewing angle

# --- Plot Planets at the Origin (Scales are vastly different, so keep small) ---
ax.plot_surface(
    earth_x / 1000, earth_y / 1000, earth_z / 1000, color="blue", alpha=0.5
)  # Scaling down the size of earth
ax.plot_surface(
    mars_x / 1000, mars_y / 1000, mars_z / 1000, color="red", alpha=0.5
)  # Scaling down the size of mars

# --- Create Animation ---
ani = FuncAnimation(
    fig, animate, frames=len(t_eval), blit=False, repeat=False
)  # Was blit=True, but changed to false because there was a strange error

# --- Save Animation (Optional) ---
# To save the animation, uncomment the following lines and install ffmpeg:
# ani.save("earth_mars_return.mp4", writer='ffmpeg', fps=30)

plt.show()
