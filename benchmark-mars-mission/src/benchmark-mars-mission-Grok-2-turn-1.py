import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define constants
earth_radius = 6371  # km
mars_radius = 3390  # km
sun_radius = 696340  # km
AU = 149597870.7  # km (Astronomical Unit)


# Function to compute positions in heliocentric coordinates
def heliocentric_position(t, a, e, T, omega, Omega, i, body_radius):
    M = 2 * np.pi * (t - T) / a**1.5  # Mean anomaly
    E = M + e * np.sin(M)  # Eccentric anomaly (first approximation)

    for _ in range(5):  # 5 iterations for better precision
        E = E - (E - e * np.sin(E) - M) / (1 - e * np.cos(E))

    x = a * (np.cos(E) - e)
    y = a * np.sqrt(1 - e**2) * np.sin(E)

    # Rotation matrices for orbital elements
    R_Omega = np.array(
        [
            [np.cos(Omega), -np.sin(Omega), 0],
            [np.sin(Omega), np.cos(Omega), 0],
            [0, 0, 1],
        ]
    )
    R_i = np.array([[1, 0, 0], [0, np.cos(i), -np.sin(i)], [0, np.sin(i), np.cos(i)]])
    R_omega = np.array(
        [
            [np.cos(omega), -np.sin(omega), 0],
            [np.sin(omega), np.cos(omega), 0],
            [0, 0, 1],
        ]
    )

    r = np.dot(R_omega, np.dot(R_i, np.dot(R_Omega, np.array([x, y, 0]))))

    # Scale to AU
    r = r / AU

    return r


# Orbital parameters - these are highly simplified
earth_orbit = {
    "a": 1,
    "e": 0.0167,
    "T": 0,
    "omega": 102.9,
    "Omega": 0,
    "i": 0,
    "radius": earth_radius / AU,
}

mars_orbit = {
    "a": 1.524,
    "e": 0.0934,
    "T": 0.27,
    "omega": 336.1,
    "Omega": 49.6,
    "i": 1.85,
    "radius": mars_radius / AU,
}

# Animation setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

# Time for animation
t = np.linspace(0, 2 * np.pi * 2, 1000)  # Two Martian years

# Initial positions for plotting Earth and Mars
earth_pos = np.array([heliocentric_position(ti, **earth_orbit) for ti in t])
mars_pos = np.array([heliocentric_position(ti, **mars_orbit) for ti in t])

# Spacecraft trajectory (very simplistic)
spacecraft_t = np.linspace(0, t[-1], 100)
spacecraft_pos = []
for ti in spacecraft_t:
    if ti < t[len(t) // 3]:  # To Mars
        spacecraft_pos.append(
            (1 - ti / t[len(t) // 3]) * earth_pos[int(ti * 1000 // t[-1])]
            + (ti / t[len(t) // 3]) * mars_pos[int(ti * 1000 // t[-1])]
        )
    elif ti < 2 * t[len(t) // 3]:  # Stay on Mars
        spacecraft_pos.append(mars_pos[int(ti * 1000 // t[-1])])
    else:  # Back to Earth
        pt = (ti - 2 * t[len(t) // 3]) / (t[-1] - 2 * t[len(t) // 3])
        spacecraft_pos.append(
            (1 - pt) * mars_pos[int(ti * 1000 // t[-1])]
            + pt * earth_pos[int(ti * 1000 // t[-1])]
        )

spacecraft_pos = np.array(spacecraft_pos)

# Plotting the planets
(earth_line,) = ax.plot([], [], [], "b-", lw=2)
(mars_line,) = ax.plot([], [], [], "r-", lw=2)
(spacecraft,) = ax.plot([], [], [], "ko", markersize=5)


# Animation function
def animate(i):
    earth_line.set_data(earth_pos[:i, 0], earth_pos[:i, 1])
    earth_line.set_3d_properties(earth_pos[:i, 2])

    mars_line.set_data(mars_pos[:i, 0], mars_pos[:i, 1])
    mars_line.set_3d_properties(mars_pos[:i, 2])

    spacecraft.set_data(spacecraft_pos[i, 0], spacecraft_pos[i, 1])
    spacecraft.set_3d_properties(spacecraft_pos[i, 2])

    ax.set_xlim(-1.6, 1.6)
    ax.set_ylim(-1.6, 1.6)
    ax.set_zlim(-1.6, 1.6)

    return earth_line, mars_line, spacecraft


anim = FuncAnimation(fig, animate, frames=len(t), interval=20, blit=True)

# Show the plot
plt.show()
