import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 天体参数 (简化模型)
AU = 1.49598e8  # 天文单位（千米）
earth_radius = 0.1  # 可视化缩放半径
mars_radius = 0.15
year_seconds = 365 * 24 * 3600

# 轨道参数（简化圆形轨道）
earth_orbit_radius = 1.0  # AU
mars_orbit_radius = 1.52  # AU
earth_angular_speed = 2 * np.pi / year_seconds
mars_angular_speed = 2 * np.pi / (1.88 * year_seconds)

# 转移轨道参数（霍曼转移）
transfer_time = 0.7 * 7.6e7  # 简化时间（秒）
return_transfer_time = 0.7 * 8.3e7

# 动画参数
total_duration = 3 * transfer_time + 5e7  # 停留时间
frames = 150
time_step = total_duration / frames

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")
ax.set_title("Earth-Mars Round Trip Mission")

# 初始化天体
(earth,) = ax.plot([], [], [], "o", color="blue", markersize=10)
(mars,) = ax.plot([], [], [], "o", color="red", markersize=12)
(ship,) = ax.plot([], [], [], "o", color="white", markersize=6)
(trail,) = ax.plot([], [], [], "-", color="gray", alpha=0.5)


def init():
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_zlim(-2, 2)
    ax.set_xlabel("X (AU)")
    ax.set_ylabel("Y (AU)")
    ax.set_zlabel("Z (AU)")
    return earth, mars, ship, trail


def update(frame):
    t = frame * time_step

    # 天体位姿
    earth_angle = earth_angular_speed * t
    mars_angle = mars_angular_speed * t
    earth_x = earth_orbit_radius * np.cos(earth_angle)
    earth_y = earth_orbit_radius * np.sin(earth_angle)

    mars_x = mars_orbit_radius * np.cos(mars_angle)
    mars_y = mars_orbit_radius * np.sin(mars_angle)

    # 航天器轨迹（分阶段）
    if t < transfer_time:  # 地球 -> 火星
        progress = t / transfer_time
        ship_x = earth_x + (mars_x - earth_x) * progress
        ship_y = earth_y + (mars_y - earth_y) * progress
    elif t < 2 * transfer_time:  # 停留期
        ship_x = mars_x
        ship_y = mars_y
    else:  # 火星 -> 地球
        progress = (t - 2 * transfer_time) / return_transfer_time
        ship_x = mars_x + (earth_x - mars_x) * progress
        ship_y = mars_y + (earth_y - mars_y) * progress

    earth.set_data([earth_x], [earth_y])
    earth.set_3d_properties(0)
    mars.set_data([mars_x], [mars_y])
    mars.set_3d_properties(0)
    ship.set_data([ship_x], [ship_y])
    ship.set_3d_properties(0)

    # 跟踪轨迹
    old_x, old_y, _ = trail.get_data_3d()
    new_x = np.append(old_x, ship_x)
    new_y = np.append(old_y, ship_y)
    new_z = np.zeros(len(new_x))
    trail.set_data(new_x, new_y)
    trail.set_3d_properties(new_z)

    return earth, mars, ship, trail


ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=50)
plt.show()
