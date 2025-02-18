import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


def plot_space_mission():
    # 设置地球和火星的轨道参数
    earth_orbit_radius = 1.0  # 天文单位
    mars_orbit_radius = 1.52  # 天文单位
    num_points = 100

    # 生成地球和火星的轨道
    theta = np.linspace(0, 2 * np.pi, num_points)
    earth_x = earth_orbit_radius * np.cos(theta)
    earth_y = earth_orbit_radius * np.sin(theta)
    mars_x = mars_orbit_radius * np.cos(theta)
    mars_y = mars_orbit_radius * np.sin(theta)

    # 初始化图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    ax.set_title("Space Mission: Earth to Mars and Back")

    # 绘制地球和火星轨道
    ax.plot(earth_x, earth_y, np.zeros(num_points), label="Earth Orbit")
    ax.plot(mars_x, mars_y, np.zeros(num_points), label="Mars Orbit")

    # 初始化航天器位置
    (spacecraft,) = ax.plot([], [], [], "ro", label="Spacecraft")

    # 初始化轨迹
    traj_x, traj_y, traj_z = [], [], []

    def update(frame):
        # 计算地球和火星的位置
        earth_pos = (
            earth_orbit_radius * np.cos(theta[frame % num_points]),
            earth_orbit_radius * np.sin(theta[frame % num_points]),
            0,
        )
        mars_pos = (
            mars_orbit_radius * np.cos(theta[(frame + 50) % num_points]),
            mars_orbit_radius * np.sin(theta[(frame + 50) % num_points]),
            0,
        )

        # 简化航天器轨迹从地球到火星再回地球
        if frame < num_points:
            # 发射到火星
            ratio = frame / num_points
            sc_x = earth_pos[0] + (mars_pos[0] - earth_pos[0]) * ratio
            sc_y = earth_pos[1] + (mars_pos[1] - earth_pos[1]) * ratio
            sc_z = 0.1 * np.sin(ratio * np.pi)  # 添加一些高度变化
        elif frame < 2 * num_points:
            # 从火星返回地球
            ratio = (frame - num_points) / num_points
            sc_x = mars_pos[0] + (earth_pos[0] - mars_pos[0]) * ratio
            sc_y = mars_pos[1] + (earth_pos[1] - mars_pos[1]) * ratio
            sc_z = 0.1 * np.sin(ratio * np.pi)
        else:
            sc_x, sc_y, sc_z = earth_pos

        traj_x.append(sc_x)
        traj_y.append(sc_y)
        traj_z.append(sc_z)

        spacecraft.set_data(sc_x, sc_y)
        spacecraft.set_3d_properties(sc_z)

        ax.plot(traj_x, traj_y, traj_z, color="r", linewidth=0.5)

        return (spacecraft,)

    ani = FuncAnimation(fig, update, frames=2 * num_points, interval=50, blit=True)

    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot_space_mission()
