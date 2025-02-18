import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# Define celestial bodies' positions (simplified)
earth_pos = np.array([0, 0, 0])
mars_pos = np.array([227.9e6, 0, 0])  # Distance in kilometers


# Define the trajectory (simplified elliptical orbit)
def interpolate_positions(start, end, steps):
    return np.linspace(start, end, steps)


# Create trajectory points
launch_to_mars = interpolate_positions(earth_pos, mars_pos, 100)
mars_to_earth = interpolate_positions(mars_pos, earth_pos, 100)

# Combine trajectories
trajectory = np.vstack((launch_to_mars, mars_to_earth))

# Create frames for animation
frames = []
for i in range(len(trajectory)):
    frame = go.Frame(
        data=[
            go.Scatter3d(
                x=trajectory[: i + 1, 0],
                y=trajectory[: i + 1, 1],
                z=trajectory[: i + 1, 2],
                mode="lines",
                line=dict(color="blue", width=4),
            ),
            go.Scatter3d(
                x=[earth_pos[0]],
                y=[earth_pos[1]],
                z=[earth_pos[2]],
                mode="markers",
                marker=dict(size=10, color="green"),
                name="Earth",
            ),
            go.Scatter3d(
                x=[mars_pos[0]],
                y=[mars_pos[1]],
                z=[mars_pos[2]],
                mode="markers",
                marker=dict(size=10, color="red"),
                name="Mars",
            ),
        ],
        name=f"frame{i}",
    )
    frames.append(frame)

# Create initial figure
fig = make_subplots(specs=[[{"type": "scatter3d"}]])

fig.add_trace(
    go.Scatter3d(
        x=[earth_pos[0]],
        y=[earth_pos[1]],
        z=[earth_pos[2]],
        mode="markers",
        marker=dict(size=10, color="green"),
        name="Earth",
    )
)
fig.add_trace(
    go.Scatter3d(
        x=[mars_pos[0]],
        y=[mars_pos[1]],
        z=[mars_pos[2]],
        mode="markers",
        marker=dict(size=10, color="red"),
        name="Mars",
    )
)
fig.add_trace(
    go.Scatter3d(
        x=[],
        y=[],
        z=[],
        mode="lines",
        line=dict(color="blue", width=4),
        name="Trajectory",
    )
)

# Add frames to figure
fig.frames = frames

# Define animation settings
animation_settings = dict(
    frame=dict(duration=50, redraw=True), fromcurrent=True, transition=dict(duration=0)
)

# Update layout with animation buttons
fig.update_layout(
    title="Earth to Mars Launch and Return Trajectory",
    scene=dict(
        xaxis_title="X (km)",
        yaxis_title="Y (km)",
        zaxis_title="Z (km)",
        aspectmode="data",
    ),
    updatemenus=[
        dict(
            type="buttons",
            buttons=[
                dict(label="Play", method="animate", args=[None, animation_settings])
            ],
        )
    ],
)

fig.show()
