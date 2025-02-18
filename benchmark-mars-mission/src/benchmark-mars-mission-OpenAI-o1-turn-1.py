import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation

"""
Function: animate_mission()
This code creates a 3D animation of a simplified Earth-Mars round trip mission.
- Earth orbit (blue)
- Mars orbit (red)
- Spacecraft moving from Earth to Mars and back to Earth
"""


def animate_mission():
    # Create figure and 3D axis
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Time steps
    frames = 100
    t = np.linspace(0, 2 * np.pi, frames)

    # Define orbits (simplified as circular/elliptical)
    earth_radius = 10
    mars_radius = 15
    earth_x = earth_radius * np.cos(t)
    earth_y = earth_radius * np.sin(t)
    earth_z = np.zeros_like(t)

    mars_x = mars_radius * np.cos(t)
    mars_y = mars_radius * np.sin(t)
    mars_z = 0.1 * mars_radius * np.sin(t)  # tilt

    # Create lines for Earth and Mars orbits
    (earth_orbit_line,) = ax.plot([], [], [], "b-", label="Earth Orbit")
    (mars_orbit_line,) = ax.plot([], [], [], "r-", label="Mars Orbit")

    # Create scatter plots for Earth, Mars, and Spacecraft
    earth_point = ax.scatter([], [], [], color="blue", s=50)
    mars_point = ax.scatter([], [], [], color="red", s=50)
    spacecraft_point = ax.scatter([], [], [], color="green", s=50, label="Spacecraft")

    # Create partial orbit for spacecraft (outbound and inbound)
    # We'll define a param (0 to 1) for each path segment
    # Outbound: Earth to Mars
    def create_trajectory(start_pos, end_pos, n=frames // 2):
        """
        Create a simple path between start_pos and end_pos
        Add a 'height_factor' to produce an arc.
        """
        t_seg = np.linspace(0, 1, n)
        height_factor = 0.3
        x_ = start_pos[0] + (end_pos[0] - start_pos[0]) * t_seg
        y_ = start_pos[1] + (end_pos[1] - start_pos[1]) * t_seg
        z_ = (
            start_pos[2]
            + (end_pos[2] - start_pos[2]) * t_seg
            + height_factor * np.sin(np.pi * t_seg)
        )
        return x_, y_, z_

    # We only create these trajectories once because the positions of Earth/Mars each frame
    # are "constrained" by time, but let's store Earth/Mars absolute paths first
    earth_positions = np.vstack((earth_x, earth_y, earth_z)).T  # shape: (frames, 3)
    mars_positions = np.vstack((mars_x, mars_y, mars_z)).T  # shape: (frames, 3)

    # This function will be updated inside update(), depending on frame.
    # But we need to hold references to spacecraft line if we want to draw it as a line
    # or just the point if we prefer. For demonstration, let's keep it as scatter only.

    # Initialize the figure limits, labels, and legend
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(-5, 5)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    ax.set_title("Earth-Mars Round Trip Simulation")

    # Initialization function for FuncAnimation
    def init():
        # Set orbits data (full orbit is static)
        earth_orbit_line.set_data(earth_x, earth_y)
        earth_orbit_line.set_3d_properties(earth_z)
        mars_orbit_line.set_data(mars_x, mars_y)
        mars_orbit_line.set_3d_properties(mars_z)
        return (
            earth_orbit_line,
            mars_orbit_line,
            earth_point,
            mars_point,
            spacecraft_point,
        )

    # Update function
    def update(frame):
        # Calculate Earth and Mars positions at current frame
        ex, ey, ez = earth_positions[frame]
        mx, my, mz = mars_positions[frame]

        # Update Earth, Mars scatter
        earth_point._offsets3d = ([ex], [ey], [ez])
        mars_point._offsets3d = ([mx], [my], [mz])

        # Spacecraft logic
        # Half frames for outbound, half for inbound
        if frame < frames // 2:
            # Outbound
            # Use Earth pos at frame=0 as start, Mars pos at frame= (frames//2) as end
            ex0, ey0, ez0 = earth_positions[0]
            mx_out, my_out, mz_out = mars_positions[frames // 2]
            x_traj, y_traj, z_traj = create_trajectory(
                (ex0, ey0, ez0), (mx_out, my_out, mz_out), n=frames // 2
            )
            # Current spacecraft position for 'frame'
            sx = x_traj[frame]
            sy = y_traj[frame]
            sz = z_traj[frame]
        else:
            # Inbound
            # Use Mars pos at frame= (frames//2) as start, Earth pos at frame=(frames-1) as end
            mx_in, my_in, mz_in = mars_positions[frames // 2]
            ex_end, ey_end, ez_end = earth_positions[frames - 1]
            x_traj, y_traj, z_traj = create_trajectory(
                (mx_in, my_in, mz_in), (ex_end, ey_end, ez_end), n=frames // 2
            )
            # for inbound frames: (frame - frames//2) in [0, frames//2)
            idx = frame - frames // 2
            sx = x_traj[idx]
            sy = y_traj[idx]
            sz = z_traj[idx]

        # Update spacecraft scatter
        spacecraft_point._offsets3d = ([sx], [sy], [sz])

        return (
            earth_orbit_line,
            mars_orbit_line,
            earth_point,
            mars_point,
            spacecraft_point,
        )

    ani = animation.FuncAnimation(
        fig, update, frames=frames, init_func=init, blit=False, interval=100
    )

    plt.show()


if __name__ == "__main__":
    animate_mission()
