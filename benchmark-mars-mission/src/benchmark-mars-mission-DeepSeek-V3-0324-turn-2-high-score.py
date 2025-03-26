import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import Normalize
import matplotlib.cm as cm

# Constants
AU = 149.6e6  # Astronomical Unit in km
earth_radius = 6371  # km
mars_radius = 3389.5  # km
sun_radius = 695700  # km (scaled down for visualization)

# Orbital parameters (simplified)
earth_orbit_radius = 1.0 * AU
mars_orbit_radius = 1.524 * AU
earth_period = 365.25  # days
mars_period = 687  # days

# Mission parameters
outbound_flight_time = 210  # days (typical Hohmann transfer)
mars_stay_time = 454  # days (until next return window)
return_flight_time = 210  # days

# Create figure and 3D axis
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection="3d")

# Set up initial view
ax.set_xlim([-2 * AU, 2 * AU])
ax.set_ylim([-2 * AU, 2 * AU])
ax.set_zlim([-0.5 * AU, 0.5 * AU])
ax.set_title("Earth-Mars Round Trip Mission")
ax.set_xlabel("X (km)")
ax.set_ylabel("Y (km)")
ax.set_zlabel("Z (km)")

# Create color maps
earth_cmap = cm.Blues
mars_cmap = cm.Reds
sun_cmap = cm.Oranges
traj_cmap = cm.viridis

# Create celestial bodies
sun = ax.scatter([0], [0], [0], color="orange", s=200, label="Sun")
(earth_orbit,) = ax.plot([], [], [], "b-", alpha=0.3, linewidth=1)
(mars_orbit,) = ax.plot([], [], [], "r-", alpha=0.3, linewidth=1)
earth = ax.scatter([], [], [], color="blue", s=50, label="Earth")
mars = ax.scatter([], [], [], color="red", s=30, label="Mars")
spacecraft = ax.scatter([], [], [], color="green", s=20, label="Spacecraft")

# Initialize trajectory lines
(outbound_traj,) = ax.plot([], [], [], "g-", alpha=0.7, linewidth=1)
(mars_stay_traj,) = ax.plot([], [], [], "g--", alpha=0.3, linewidth=0.5)
(return_traj,) = ax.plot([], [], [], "g-", alpha=0.7, linewidth=1)

# Initialize text annotations
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)
phase_text = ax.text2D(0.02, 0.90, "", transform=ax.transAxes)


# Calculate orbits
def calculate_orbits():
    theta = np.linspace(0, 2 * np.pi, 100)

    # Earth orbit
    earth_x = earth_orbit_radius * np.cos(theta)
    earth_y = earth_orbit_radius * np.sin(theta)
    earth_z = np.zeros_like(theta)

    # Mars orbit (slightly inclined for 3D effect)
    mars_x = mars_orbit_radius * np.cos(theta)
    mars_y = mars_orbit_radius * np.sin(theta)
    mars_z = 0.1 * AU * np.sin(theta)  # Small inclination

    return earth_x, earth_y, earth_z, mars_x, mars_y, mars_z


# Calculate positions over time
def calculate_positions(total_days=1000, steps=500):
    days = np.linspace(0, total_days, steps)

    # Earth positions
    earth_angle = 2 * np.pi * days / earth_period
    earth_x = earth_orbit_radius * np.cos(earth_angle)
    earth_y = earth_orbit_radius * np.sin(earth_angle)
    earth_z = np.zeros_like(earth_x)

    # Mars positions
    mars_angle = 2 * np.pi * days / mars_period
    mars_x = mars_orbit_radius * np.cos(mars_angle)
    mars_y = mars_orbit_radius * np.sin(mars_angle)
    mars_z = 0.1 * AU * np.sin(mars_angle)

    # Find launch and return windows
    launch_day = 0
    arrival_day = launch_day + outbound_flight_time
    return_day = arrival_day + mars_stay_time
    earth_return_day = return_day + return_flight_time

    # Spacecraft trajectory
    sc_x, sc_y, sc_z = np.zeros_like(days), np.zeros_like(days), np.zeros_like(days)
    mission_phase = np.zeros_like(
        days, dtype=int
    )  # 0=pre-launch, 1=outbound, 2=stay, 3=return

    for i, day in enumerate(days):
        if day < launch_day:
            # Pre-launch - spacecraft on Earth
            sc_x[i] = earth_x[i]
            sc_y[i] = earth_y[i]
            sc_z[i] = earth_z[i]
            mission_phase[i] = 0
        elif day < arrival_day:
            # Outbound flight - interpolate between Earth and Mars positions at launch/arrival
            progress = (day - launch_day) / outbound_flight_time
            launch_idx = np.argmin(np.abs(days - launch_day))
            arrival_idx = np.argmin(np.abs(days - arrival_day))

            sc_x[i] = earth_x[launch_idx] + progress * (
                mars_x[arrival_idx] - earth_x[launch_idx]
            )
            sc_y[i] = earth_y[launch_idx] + progress * (
                mars_y[arrival_idx] - earth_y[launch_idx]
            )
            sc_z[i] = earth_z[launch_idx] + progress * (
                mars_z[arrival_idx] - earth_z[launch_idx]
            )
            mission_phase[i] = 1
        elif day < return_day:
            # Stay at Mars
            mars_idx = np.argmin(np.abs(days - day))
            sc_x[i] = mars_x[mars_idx]
            sc_y[i] = mars_y[mars_idx]
            sc_z[i] = mars_z[mars_idx]
            mission_phase[i] = 2
        elif day < earth_return_day:
            # Return flight - interpolate between Mars and Earth positions at return/arrival
            progress = (day - return_day) / return_flight_time
            return_idx = np.argmin(np.abs(days - return_day))
            earth_arrival_idx = np.argmin(np.abs(days - earth_return_day))

            sc_x[i] = mars_x[return_idx] + progress * (
                earth_x[earth_arrival_idx] - mars_x[return_idx]
            )
            sc_y[i] = mars_y[return_idx] + progress * (
                earth_y[earth_arrival_idx] - mars_y[return_idx]
            )
            sc_z[i] = mars_z[return_idx] + progress * (
                earth_z[earth_arrival_idx] - mars_z[return_idx]
            )
            mission_phase[i] = 3
        else:
            # Mission complete - spacecraft back on Earth
            earth_idx = np.argmin(np.abs(days - day))
            sc_x[i] = earth_x[earth_idx]
            sc_y[i] = earth_y[earth_idx]
            sc_z[i] = earth_z[earth_idx]
            mission_phase[i] = 4

    return (
        days,
        earth_x,
        earth_y,
        earth_z,
        mars_x,
        mars_y,
        mars_z,
        sc_x,
        sc_y,
        sc_z,
        mission_phase,
    )


# Calculate all positions
(
    earth_orbit_x,
    earth_orbit_y,
    earth_orbit_z,
    mars_orbit_x,
    mars_orbit_y,
    mars_orbit_z,
) = calculate_orbits()
(
    days,
    earth_x,
    earth_y,
    earth_z,
    mars_x,
    mars_y,
    mars_z,
    sc_x,
    sc_y,
    sc_z,
    mission_phase,
) = calculate_positions()


# Animation update function
def update(frame):
    # Update planet positions
    earth._offsets3d = ([earth_x[frame]], [earth_y[frame]], [earth_z[frame]])
    mars._offsets3d = ([mars_x[frame]], [mars_y[frame]], [mars_z[frame]])

    # Update spacecraft position
    spacecraft._offsets3d = ([sc_x[frame]], [sc_y[frame]], [sc_z[frame]])

    # Update orbits
    earth_orbit.set_data(earth_orbit_x, earth_orbit_y)
    earth_orbit.set_3d_properties(earth_orbit_z)
    mars_orbit.set_data(mars_orbit_x, mars_orbit_y)
    mars_orbit.set_3d_properties(mars_orbit_z)

    # Update trajectory lines
    current_phase = mission_phase[frame]

    # Outbound trajectory (phase 1)
    outbound_mask = (mission_phase == 1) & (np.arange(len(mission_phase)) <= frame)
    outbound_traj.set_data(sc_x[outbound_mask], sc_y[outbound_mask])
    outbound_traj.set_3d_properties(sc_z[outbound_mask])

    # Mars stay trajectory (phase 2)
    stay_mask = (mission_phase == 2) & (np.arange(len(mission_phase)) <= frame)
    mars_stay_traj.set_data(sc_x[stay_mask], sc_y[stay_mask])
    mars_stay_traj.set_3d_properties(sc_z[stay_mask])

    # Return trajectory (phase 3)
    return_mask = (mission_phase == 3) & (np.arange(len(mission_phase)) <= frame)
    return_traj.set_data(sc_x[return_mask], sc_y[return_mask])
    return_traj.set_3d_properties(sc_z[return_mask])

    # Update text
    day = days[frame]
    time_text.set_text(f"Mission Day: {day:.1f}")

    phase_names = [
        "Pre-launch",
        "Outbound to Mars",
        "Stay at Mars",
        "Return to Earth",
        "Mission Complete",
    ]
    phase_text.set_text(f"Phase: {phase_names[current_phase]}")

    # Adjust view for better perspective
    ax.view_init(elev=20, azim=day / 5)

    return (
        earth,
        mars,
        spacecraft,
        earth_orbit,
        mars_orbit,
        outbound_traj,
        mars_stay_traj,
        return_traj,
        time_text,
        phase_text,
    )


# Create animation
ani = FuncAnimation(fig, update, frames=len(days), interval=50, blit=False)

# Add legend
ax.legend(loc="upper right")

plt.tight_layout()
plt.show()

# To save the animation (uncomment to use)
# ani.save('earth_mars_roundtrip.mp4', writer='ffmpeg', fps=30, dpi=300)
