import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def earth_mars_round_trip_animation():
    """
    This function creates an animated 3D plot of a launch from Earth,
    landing on Mars, and then returning to Earth at the next launch window.
    """

    # 1. Set up orbital parameters (simplified)
    earth_orbit_radius = 1.0
    mars_orbit_radius = 1.52

    earth_orbital_period = 365  # days
    mars_orbital_period = 687  # days

    # 2. Define the total frames for the animation
    total_days = 1000

    # 3. Calculate angular velocity (radians/day) for Earth and Mars
    earth_omega = 2.0 * np.pi / earth_orbital_period
    mars_omega = 2.0 * np.pi / mars_orbital_period

    # 4. Prepare figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # 5. Compute Earth and Mars positions
    earth_positions = []
    mars_positions = []

    for day in range(total_days):
        # Earth position
        earth_angle = earth_omega * day
        ex = earth_orbit_radius * np.cos(earth_angle)
        ey = earth_orbit_radius * np.sin(earth_angle)
        ez = 0.0
        earth_positions.append([ex, ey, ez])

        # Mars position
        mars_angle = mars_omega * day
        mx = mars_orbit_radius * np.cos(mars_angle)
        my = mars_orbit_radius * np.sin(mars_angle)
        mz = 0.0
        mars_positions.append([mx, my, mz])

    earth_positions = np.array(earth_positions)
    mars_positions = np.array(mars_positions)

    # 6. Spaceship trajectory (simplified interpolation)
    def spaceship_trajectory(day):
        # Segment A: Launch to Mars (days 0 ~ 200)
        if 0 <= day < 200:
            factor = day / 200.0
            sx = earth_positions[0, 0] + factor * (
                mars_positions[200, 0] - earth_positions[0, 0]
            )
            sy = earth_positions[0, 1] + factor * (
                mars_positions[200, 1] - earth_positions[0, 1]
            )
            sz = earth_positions[0, 2] + factor * (
                mars_positions[200, 2] - earth_positions[0, 2]
            )
            return sx, sy, sz

        # Segment B: On Mars (~ days 200 ~ 400)
        elif 200 <= day < 400:
            return (
                mars_positions[day, 0],
                mars_positions[day, 1],
                mars_positions[day, 2],
            )

        # Segment C: Return to Earth (~ days 400 ~ 600)
        elif 400 <= day < 600:
            factor = (day - 400) / 200.0
            sx = mars_positions[400, 0] + factor * (
                earth_positions[600, 0] - mars_positions[400, 0]
            )
            sy = mars_positions[400, 1] + factor * (
                earth_positions[600, 1] - mars_positions[400, 1]
            )
            sz = mars_positions[400, 2] + factor * (
                earth_positions[600, 2] - mars_positions[400, 2]
            )
            return sx, sy, sz

        else:
            # Segment D: On Earth again (~ days 600+)
            return (
                earth_positions[day, 0],
                earth_positions[day, 1],
                earth_positions[day, 2],
            )

    # 7. Plot orbits
    ax.plot(
        earth_positions[:, 0],
        earth_positions[:, 1],
        earth_positions[:, 2],
        color="blue",
        linewidth=0.5,
        label="Earth Orbit",
    )
    ax.plot(
        mars_positions[:, 0],
        mars_positions[:, 1],
        mars_positions[:, 2],
        color="red",
        linewidth=0.5,
        label="Mars Orbit",
    )

    # 8. Create plot objects for Earth, Mars, and spaceship
    (earth_point,) = ax.plot([], [], [], "bo", markersize=6, label="Earth")
    (mars_point,) = ax.plot([], [], [], "ro", markersize=6, label="Mars")
    (spaceship_point,) = ax.plot([], [], [], "go", markersize=6, label="Spaceship")

    # 9. Init function
    def init():
        ax.set_xlim3d(-2.0, 2.0)
        ax.set_ylim3d(-2.0, 2.0)
        ax.set_zlim3d(-1.0, 1.0)
        ax.set_xlabel("X (AU)")
        ax.set_ylabel("Y (AU)")
        ax.set_zlabel("Z (AU)")
        ax.set_title("Earth-Mars Round Trip Animation (Simplified)")

        earth_point.set_data([], [])
        earth_point.set_3d_properties([])
        mars_point.set_data([], [])
        mars_point.set_3d_properties([])
        spaceship_point.set_data([], [])
        spaceship_point.set_3d_properties([])
        return earth_point, mars_point, spaceship_point

    # 10. Update function
    def update(frame):
        # Make sure to pass list or array to set_data
        earth_point.set_data([earth_positions[frame, 0]], [earth_positions[frame, 1]])
        earth_point.set_3d_properties([earth_positions[frame, 2]])

        mars_point.set_data([mars_positions[frame, 0]], [mars_positions[frame, 1]])
        mars_point.set_3d_properties([mars_positions[frame, 2]])

        sx, sy, sz = spaceship_trajectory(frame)
        spaceship_point.set_data([sx], [sy])
        spaceship_point.set_3d_properties([sz])

        return earth_point, mars_point, spaceship_point

    anim = FuncAnimation(
        fig, update, frames=total_days, init_func=init, interval=20, blit=True
    )
    plt.legend()
    plt.show()


if __name__ == "__main__":
    earth_mars_round_trip_animation()
