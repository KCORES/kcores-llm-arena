import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from scipy.interpolate import interp1d

# Constants
G = 6.67430e-11  # Gravitational constant [m^3 kg^-1 s^-2]
M_earth = 5.972e24  # Mass of Earth [kg]
M_mars = 6.4185e23  # Mass of Mars [kg]
r_earth = 6371000  # Radius of Earth [m]
r_mars = 3389500  # Radius of Mars [m]

# Initial conditions
v_earth_escape = np.sqrt(2 * G * M_earth / r_earth)  # Escape velocity from Earth [m/s]
v_mars_orbit = np.sqrt(G * M_mars / (1.524e11))  # Orbital velocity around Mars [m/s]

# Trajectory parameters
launch_angle = np.radians(45)  # Launch angle from Earth's surface
mars_distance = 2.25e11  # Distance from Earth to Mars at launch [m]
earth_mars_transfer_time = 225 * 24 * 3600  # Transfer time from Earth to Mars [s]

# Time parameters
t_launch_earth = 0
t_arrive_mars = earth_mars_transfer_time
t_leave_mars = t_arrive_mars + 30 * 24 * 3600  # Stay on Mars for 30 days
t_arrive_earth = t_leave_mars + earth_mars_transfer_time

# Launch window
launch_window = 26 * 24 * 3600  # Launch window [s]


# Generate trajectory
def generate_trajectory(
    t, r_earth, M_earth, M_mars, r_mars, launch_angle, v_earth_escape
):
    # Earth to Mars
    t_earth_mars = np.linspace(0, t_arrive_mars, 1000)
    r_earth_mars = np.array(
        [
            np.cos(2 * np.pi * t_earth_mars / earth_mars_transfer_time) * mars_distance,
            np.sin(2 * np.pi * t_earth_mars / earth_mars_transfer_time) * mars_distance,
            np.zeros_like(t_earth_mars),
        ]
    )

    # Mars to Earth
    t_mars_earth = np.linspace(t_arrive_mars, t_arrive_earth, 1000)
    r_mars_earth = np.array(
        [
            (t_mars_earth - t_arrive_mars)
            / (t_arrive_earth - t_arrive_mars)
            * mars_distance
            * 1.2,
            np.sin(
                2
                * np.pi
                * (t_mars_earth - t_arrive_mars)
                / (t_arrive_earth - t_arrive_mars)
            )
            * mars_distance,
            np.zeros_like(t_mars_earth),
        ]
    )

    # Interpolate trajectory
    trajectory = np.concatenate((r_earth_mars, r_mars_earth), axis=1)

    return trajectory


# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Generate trajectory
trajectory = generate_trajectory(
    np.linspace(0, t_arrive_earth, 2000),
    r_earth,
    M_earth,
    M_mars,
    r_mars,
    launch_angle,
    v_earth_escape,
)


# Animate trajectory
def animate(i):
    ax.clear()
    ax.plot(trajectory[0, :i], trajectory[1, :i], trajectory[2, :i], "b-")
    ax.scatter(0, 0, 0, c="g", marker="o")  # Earth
    ax.scatter(
        mars_distance * np.cos(np.radians(45)),
        mars_distance * np.sin(np.radians(45)),
        0,
        c="r",
        marker="o",
    )  # Mars


ani = animation.FuncAnimation(fig, animate, frames=len(trajectory[0]), interval=50)

plt.show()
