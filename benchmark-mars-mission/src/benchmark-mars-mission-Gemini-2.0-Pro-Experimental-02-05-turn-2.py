import plotly.graph_objects as go
import numpy as np
from datetime import datetime, timedelta
from scipy.constants import astronomical_unit as AU  # Use astronomical unit

# --- Constants and Orbital Parameters ---

# Gravitational parameters (simplified for visualization)
mu_sun = 1.32712440018e20  # Standard gravitational parameter of the Sun (m^3/s^2) - used for orbital period calculation

# Earth (simplified, circular orbits)
earth_radius = 6371000  # m
earth_orbital_radius = 1.0 * AU  # m
earth_orbital_period = 365.25 * 24 * 3600  # seconds
earth_color = "blue"

# Mars (simplified, circular orbits)
mars_radius = 3389500  # m
mars_orbital_radius = 1.524 * AU  # m
mars_orbital_period = 686.98 * 24 * 3600  # seconds
mars_color = "red"

# Spacecraft (initial values)
spacecraft_size = 5e6  # Size for visualization (adjust as needed)
launch_date = datetime(2024, 1, 1)  # Example launch date
transfer_duration_1 = 250 * 24 * 3600  # Example Hohmann transfer time to Mars (seconds)
stay_duration = 500 * 24 * 3600  # Example wait until the next launch window
transfer_duration_2 = 250 * 24 * 3600  # Example Hohmann transfer time to Earth(seconds)
spacecraft_color = "white"

# --- Helper Functions ---


def orbital_position(radius, period, time):
    """Calculates the orbital position (x, y, z) at a given time."""
    angle = 2 * np.pi * (time % period) / period
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = 0
    return x, y, z


def hohmann_transfer(r1, r2, mu):
    """Calculates the delta-v and time of flight for a Hohmann transfer."""
    # Semi-major axis of the transfer ellipse
    a = (r1 + r2) / 2

    # Velocities at periapsis and apoapsis
    v1 = np.sqrt(mu * (2 / r1 - 1 / a))
    v2 = np.sqrt(mu * (2 / r2 - 1 / a))

    # Delta-vs at departure and arrival
    delta_v_departure = v1 - np.sqrt(mu / r1)  # added
    delta_v_arrival = np.sqrt(mu / r2) - v2  # added

    # Time of flight (half of the orbital period)
    tof = np.pi * np.sqrt(a**3 / mu)

    return delta_v_departure, delta_v_arrival, tof  # Return delta-v and tof


# --- Animation Setup ---

total_duration = (
    transfer_duration_1 + stay_duration + transfer_duration_2
)  # Total animation time
num_frames = 500  # Number of animation frames
time_step = total_duration / num_frames
times = np.linspace(0, total_duration, num_frames)

# --- Create Planet Trajectories ---
earth_positions = np.array(
    [orbital_position(earth_orbital_radius, earth_orbital_period, t) for t in times]
)
mars_positions = np.array(
    [orbital_position(mars_orbital_radius, mars_orbital_period, t) for t in times]
)


# --- Create Spacecraft Trajectory ---

spacecraft_positions = []
current_time = 0

for i in range(num_frames):
    t = times[i]

    if t <= transfer_duration_1:  # Launch to Mars
        # Hohmann Transfer to Mars (simplified for animation)
        progress = t / transfer_duration_1
        # Interpolate between Earth and Mars orbits
        interp_radius = earth_orbital_radius + progress * (
            mars_orbital_radius - earth_orbital_radius
        )

        # Adjusted angle calculation for transfer ellipse
        start_angle = (
            2 * np.pi * (current_time % earth_orbital_period) / earth_orbital_period
        )
        end_angle = (
            2
            * np.pi
            * ((current_time + transfer_duration_1) % mars_orbital_period)
            / mars_orbital_period
        )

        angle = start_angle + progress * (end_angle - start_angle)
        if (end_angle - start_angle) < 0:
            angle = start_angle + progress * (end_angle + 2 * np.pi - start_angle)

        x = interp_radius * np.cos(angle)
        y = interp_radius * np.sin(angle)
        z = 0

    elif t <= transfer_duration_1 + stay_duration:
        # Stay at mars
        mars_arrival_time = current_time + transfer_duration_1
        x, y, z = orbital_position(mars_orbital_radius, mars_orbital_period, t)

    else:  # Return to Earth.
        # Hohmann Transfer to Earth (simplified)
        progress = (t - (transfer_duration_1 + stay_duration)) / transfer_duration_2
        interp_radius = mars_orbital_radius + progress * (
            earth_orbital_radius - mars_orbital_radius
        )

        # Adjusted angle calculation
        start_angle = (
            2
            * np.pi
            * (
                (current_time + transfer_duration_1 + stay_duration)
                % mars_orbital_period
            )
            / mars_orbital_period
        )
        end_angle = (
            2
            * np.pi
            * (
                (
                    current_time
                    + transfer_duration_1
                    + stay_duration
                    + transfer_duration_2
                )
                % earth_orbital_period
            )
            / earth_orbital_period
        )

        angle = start_angle + progress * (end_angle - start_angle)
        if (end_angle - start_angle) < 0:
            angle = start_angle + progress * (end_angle + 2 * np.pi - start_angle)

        x = interp_radius * np.cos(angle)
        y = interp_radius * np.sin(angle)
        z = 0
    spacecraft_positions.append((x, y, z))

    current_time = t

spacecraft_positions = np.array(spacecraft_positions)


# --- Create Animation Frames ---

frames = []
for i in range(num_frames):
    earth_x, earth_y, earth_z = earth_positions[i]
    mars_x, mars_y, mars_z = mars_positions[i]
    spacecraft_x, spacecraft_y, spacecraft_z = spacecraft_positions[i]

    frame = go.Frame(
        data=[
            go.Scatter3d(
                x=[earth_x],
                y=[earth_y],
                z=[earth_z],
                mode="markers",
                marker=dict(size=10, color=earth_color),
            ),
            go.Scatter3d(
                x=[mars_x],
                y=[mars_y],
                z=[mars_z],
                mode="markers",
                marker=dict(size=8, color=mars_color),
            ),
            go.Scatter3d(
                x=[spacecraft_x],
                y=[spacecraft_y],
                z=[spacecraft_z],
                mode="markers",
                marker=dict(size=4, color=spacecraft_color),
            ),
            go.Scatter3d(
                x=earth_positions[: i + 1, 0],
                y=earth_positions[: i + 1, 1],
                z=earth_positions[: i + 1, 2],
                mode="lines",
                line=dict(color=earth_color, width=2),
            ),  # Earth orbit
            go.Scatter3d(
                x=mars_positions[: i + 1, 0],
                y=mars_positions[: i + 1, 1],
                z=mars_positions[: i + 1, 2],
                mode="lines",
                line=dict(color=mars_color, width=2),
            ),  # Mars orbit
            go.Scatter3d(
                x=spacecraft_positions[: i + 1, 0],
                y=spacecraft_positions[: i + 1, 1],
                z=spacecraft_positions[: i + 1, 2],
                mode="lines",
                line=dict(color=spacecraft_color, width=2),
            ),  # spacecraft path
        ],
        name=str(i),
    )
    frames.append(frame)

# --- Create Initial Figure ---

fig = go.Figure(
    data=[
        go.Scatter3d(
            x=[earth_positions[0, 0]],
            y=[earth_positions[0, 1]],
            z=[earth_positions[0, 2]],
            mode="markers",
            marker=dict(size=10, color=earth_color),
            name="Earth",
        ),
        go.Scatter3d(
            x=[mars_positions[0, 0]],
            y=[mars_positions[0, 1]],
            z=[mars_positions[0, 2]],
            mode="markers",
            marker=dict(size=8, color=mars_color),
            name="Mars",
        ),
        go.Scatter3d(
            x=[spacecraft_positions[0, 0]],
            y=[spacecraft_positions[0, 1]],
            z=[spacecraft_positions[0, 2]],
            mode="markers",
            marker=dict(size=4, color=spacecraft_color),
            name="Spacecraft",
        ),
        go.Scatter3d(
            x=earth_positions[:, 0],
            y=earth_positions[:, 1],
            z=earth_positions[:, 2],
            mode="lines",
            line=dict(color=earth_color, width=2),
            showlegend=False,
        ),  # Earth orbit
        go.Scatter3d(
            x=mars_positions[:, 0],
            y=mars_positions[:, 1],
            z=mars_positions[:, 2],
            mode="lines",
            line=dict(color=mars_color, width=2),
            showlegend=False,
        ),  # Mars orbit
        go.Scatter3d(
            x=spacecraft_positions[:, 0],
            y=spacecraft_positions[:, 1],
            z=spacecraft_positions[:, 2],
            mode="lines",
            line=dict(color=spacecraft_color, width=2),
            showlegend=False,
        ),  # spacecraft path
    ],
    layout=go.Layout(
        scene=dict(
            xaxis=dict(range=[-2.5 * AU, 2.5 * AU]),
            yaxis=dict(range=[-2.5 * AU, 2.5 * AU]),
            zaxis=dict(range=[-1 * AU, 1 * AU]),
            aspectmode="cube",
        ),
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                buttons=[
                    dict(
                        label="Play",
                        method="animate",
                        args=[
                            None,
                            dict(
                                frame=dict(duration=20, redraw=True), fromcurrent=True
                            ),
                        ],
                    )
                ],
            )
        ],
        title="Earth-Mars-Earth Mission",
    ),
    frames=frames,
)

fig.show()
