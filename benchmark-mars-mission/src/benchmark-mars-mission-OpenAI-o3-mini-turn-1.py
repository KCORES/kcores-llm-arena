import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


# Function: quadratic_bezier
def quadratic_bezier(P0, P1, P2, t):
    """
    Compute a point on a quadratic Bezier curve.
    Parameters:
        P0, P1, P2: control points (numpy arrays)
        t: parameter in [0, 1]
    Returns:
        A numpy array representing the point on the curve.
    """
    return (1 - t) ** 2 * P0 + 2 * t * (1 - t) * P1 + t**2 * P2


# Define Earth and Mars parameters
r_earth = 1.0  # Earth radius
r_mars = 0.5  # Mars radius
center_earth = np.array([0, 0, 0])
center_mars = np.array([20, 0, 0])

# Define trajectory control points for Earth->Mars flight
P0_em = center_earth + np.array(
    [r_earth, 0, 0]
)  # Earth launch point on surface (facing Mars)
P2_em = center_mars - np.array(
    [r_mars, 0, 0]
)  # Mars landing point on surface (facing Earth)
P1_em = np.array(
    [(P0_em[0] + P2_em[0]) / 2, 10, 8]
)  # Control point to form an arched trajectory

# Define trajectory control points for Mars->Earth return flight
P0_me = P2_em  # Mars departure point (same as landing point)
P2_me = center_earth + np.array(
    [r_earth, 0, 0]
)  # Earth arrival point (same as launch point)
P1_me = np.array(
    [(P0_me[0] + P2_me[0]) / 2, -10, -8]
)  # Control point for a different return arc

# Generate trajectory points using the quadratic Bezier curve
n_points = 200  # Number of points per trajectory segment
t_values = np.linspace(0, 1, n_points)
trajectory_em = np.array([quadratic_bezier(P0_em, P1_em, P2_em, t) for t in t_values])
trajectory_me = np.array([quadratic_bezier(P0_me, P1_me, P2_me, t) for t in t_values])

# Define hold frames at Mars (pause before return flight)
hold_frames = 20

# Combine both trajectory segments and the hold period
trajectory_full = np.concatenate(
    [
        trajectory_em,  # Earth -> Mars flight
        np.repeat(
            trajectory_em[-1][np.newaxis, :], hold_frames, axis=0
        ),  # Pause at Mars
        trajectory_me,  # Mars -> Earth return flight
    ],
    axis=0,
)

total_frames = trajectory_full.shape[0]


# Function: create_sphere
def create_sphere(center, radius, num_points=30):
    """
    Generate coordinates for a sphere surface.
    Parameters:
        center: numpy array representing the center.
        radius: radius of the sphere.
        num_points: resolution for the sphere mesh (default: 30).
    Returns:
        Meshgrid arrays (x, y, z) for the sphere surface.
    """
    u, v = np.mgrid[
        0 : 2 * np.pi : complex(0, num_points), 0 : np.pi : complex(0, num_points)
    ]
    x = center[0] + radius * np.cos(u) * np.sin(v)
    y = center[1] + radius * np.sin(u) * np.sin(v)
    z = center[2] + radius * np.cos(v)
    return x, y, z


# Create sphere data for Earth and Mars
x_e, y_e, z_e = create_sphere(center_earth, r_earth)
x_m, y_m, z_m = create_sphere(center_mars, r_mars)

# Setup the 3D plot figure and axis
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Plot Earth and Mars surfaces
ax.plot_surface(x_e, y_e, z_e, color="b", alpha=0.3)
ax.plot_surface(x_m, y_m, z_m, color="r", alpha=0.3)

# Plot static trajectory paths for reference
ax.plot(
    trajectory_em[:, 0],
    trajectory_em[:, 1],
    trajectory_em[:, 2],
    "g--",
    label="Earth to Mars",
)
ax.plot(
    trajectory_me[:, 0],
    trajectory_me[:, 1],
    trajectory_me[:, 2],
    "m--",
    label="Mars to Earth",
)

# Set plot limits and labels
ax.set_xlim(-5, 25)
ax.set_ylim(-15, 15)
ax.set_zlim(-15, 15)
ax.set_xlabel("X (Space)")
ax.set_ylabel("Y (Space)")
ax.set_zlabel("Z (Space)")
ax.set_title("Interplanetary Launch: Earth -> Mars -> Earth")
ax.legend()

# Initialize the rocket marker and its flight path
(rocket_marker,) = ax.plot([], [], [], "ko", markersize=5, label="Rocket")
(rocket_path,) = ax.plot([], [], [], "k-", linewidth=2)


# Function: update
def update(frame):
    """
    Update function for animation.
    Parameters:
        frame: current animation frame index.
    Returns:
        Updated artists (rocket marker and flight path).
    """
    # Get current rocket position along the full trajectory
    pos = trajectory_full[frame]
    rocket_marker.set_data(pos[0], pos[1])
    rocket_marker.set_3d_properties(pos[2])

    # Update the path (traced trajectory) up to the current frame
    path_data = trajectory_full[: frame + 1]
    rocket_path.set_data(path_data[:, 0], path_data[:, 1])
    rocket_path.set_3d_properties(path_data[:, 2])

    return rocket_marker, rocket_path


# Create the animation object
anim = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=False)

# Display the animated plot
plt.show()
