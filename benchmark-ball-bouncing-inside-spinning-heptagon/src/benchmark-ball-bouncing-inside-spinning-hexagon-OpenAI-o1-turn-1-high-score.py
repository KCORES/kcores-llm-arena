import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

HEPTAGON_CENTER_X = WINDOW_WIDTH // 2
HEPTAGON_CENTER_Y = WINDOW_HEIGHT // 2
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 300  # Radius for the circumscribed circle of the heptagon

NUM_BALLS = 20
BALL_RADIUS = 15
GRAVITY = 400.0  # px/s^2
AIR_FRICTION = 0.99  # Horizontal friction factor each frame
SPIN_FRICTION = 0.99  # Spin friction (rotational friction)
TIME_STEP = 1 / 60  # Simulation time step (seconds per frame)
ROTATION_SPEED = 72.0  # Heptagon rotation speed (degrees per second)
RESTITUTION_BALL = 0.9  # Coefficient of restitution for ball-ball collisions
RESTITUTION_WALL = 0.9  # Coefficient of restitution for ball-wall collisions

COLORS = [
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ec6800",
    "#ee7800",
    "#eb6238",
    "#ea5506",
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
    "#db8449",
    "#d66a35",
]


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    spin: float
    color: str
    number: int


class SpinningHeptagonSimulation:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(
            self.master, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white"
        )
        self.canvas.pack()

        # 初始化七邊形旋轉角度
        self.heptagon_angle = 0.0

        # 建立球的清單
        self.balls: List[Ball] = []
        self.init_balls()

        # tkinter 的動畫更新
        self.update_canvas()

    def init_balls(self):
        # 初始時球從中心跌落，給予微小隨機位移 & 速度
        np.random.seed(42)  # for reproducible demo
        for i in range(NUM_BALLS):
            # 讓球初始位置都在中心 (加上微小雜訊)
            x = HEPTAGON_CENTER_X + np.random.uniform(-1, 1)
            y = HEPTAGON_CENTER_Y + np.random.uniform(-1, 1)
            vx = np.random.uniform(-50, 50)
            vy = np.random.uniform(-50, 50)
            spin = np.random.uniform(-10, 10)
            color = COLORS[i % len(COLORS)]
            ball = Ball(x, y, vx, vy, BALL_RADIUS, spin, color, i + 1)
            self.balls.append(ball)

    def get_heptagon_points(self, angle_deg: float):
        """
        依據目前旋轉角度，計算七邊形頂點座標。
        回傳一個 [(x1, y1), (x2, y2), ..., (x7, y7)] 的列表。
        """
        points = []
        angle_rad_offset = math.radians(angle_deg)
        for i in range(HEPTAGON_SIDES):
            # 這裡假設正七邊形的頂點分布在圓周上
            theta = 2 * math.pi * i / HEPTAGON_SIDES + angle_rad_offset
            px = HEPTAGON_CENTER_X + HEPTAGON_RADIUS * math.cos(theta)
            py = HEPTAGON_CENTER_Y + HEPTAGON_RADIUS * math.sin(theta)
            points.append((px, py))
        return points

    def update_physics(self):
        # 更新七邊形旋轉角度
        self.heptagon_angle += ROTATION_SPEED * TIME_STEP
        if self.heptagon_angle >= 360.0:
            self.heptagon_angle -= 360.0

        # 計算七邊形頂點 (當作碰撞邊)
        hept_points = self.get_heptagon_points(self.heptagon_angle)

        # 每顆球都受到重力影響
        for ball in self.balls:
            # 重力只作用在 vy
            ball.vy += GRAVITY * TIME_STEP

            # 速度進行摩擦衰減 (空氣阻力)
            ball.vx *= AIR_FRICTION
            ball.vy *= AIR_FRICTION

            # 旋轉衰減
            ball.spin *= SPIN_FRICTION

        # 處理球與球碰撞
        self.handle_ball_collisions()

        # 更新球位置並處理球與牆碰撞
        self.handle_wall_collisions(hept_points)

    def handle_ball_collisions(self):
        # 兩兩檢查球是否碰撞
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist = math.hypot(dx, dy)
                if dist < (b1.radius + b2.radius):
                    # 兩球已碰撞，進行彈性碰撞處理

                    # 先確保兩球略微分開，避免卡住
                    overlap = (b1.radius + b2.radius) - dist
                    # normalize dx, dy
                    nx = dx / dist
                    ny = dy / dist
                    # 平均分配重疊的調整量
                    b1.x -= nx * overlap * 0.5
                    b1.y -= ny * overlap * 0.5
                    b2.x += nx * overlap * 0.5
                    b2.y += ny * overlap * 0.5

                    # 計算彈性碰撞的速度交換
                    # 分離相對於法向量 (nx, ny) 的速度
                    v1n = b1.vx * nx + b1.vy * ny
                    v2n = b2.vx * nx + b2.vy * ny
                    # 彈性碰撞後法向分量交換(依照動量守恆、彈性係數)
                    # 這裡簡化為兩球質量相同狀況
                    new_v1n = v2n
                    new_v2n = v1n
                    # 加入碰撞恢復係數
                    new_v1n *= RESTITUTION_BALL
                    new_v2n *= RESTITUTION_BALL

                    # 計算球之間的相對切向速度，用來簡單模擬 spin 變化
                    # (實際上應更完整計算碰撞產生的切向衝量)
                    vt1 = b1.vx * -ny + b1.vy * nx
                    vt2 = b2.vx * -ny + b2.vy * nx
                    spin_impact = 0.05 * (vt1 - vt2)
                    b1.spin += spin_impact
                    b2.spin -= spin_impact

                    # 把新的法向量速度更新回 vx, vy
                    b1.vx = new_v1n * nx + vt1 * -ny
                    b1.vy = new_v1n * ny + vt1 * nx
                    b2.vx = new_v2n * nx + vt2 * -ny
                    b2.vy = new_v2n * ny + vt2 * nx

    def handle_wall_collisions(self, hept_points):
        # 先組合七邊形的邊 (edge): (p[i], p[i+1])
        edges = []
        for i in range(len(hept_points)):
            p1 = hept_points[i]
            p2 = hept_points[(i + 1) % len(hept_points)]
            edges.append((p1, p2))

        for ball in self.balls:
            # 先更新球位置
            ball.x += ball.vx * TIME_STEP
            ball.y += ball.vy * TIME_STEP
            # 檢查與每條邊的碰撞
            for edge in edges:
                p1, p2 = edge
                # 邊向量
                ex = p2[0] - p1[0]
                ey = p2[1] - p1[1]
                # 邊的長度
                edge_len = math.hypot(ex, ey)
                # 單位向量
                if edge_len == 0:
                    continue
                exn = ex / edge_len
                eyn = ey / edge_len

                # 計算從 p1 到球中心的向量
                dx = ball.x - p1[0]
                dy = ball.y - p1[1]
                # 對邊投影的大小
                proj = dx * exn + dy * eyn
                # 投影點(最近點)
                if proj < 0:
                    # 最近點就是 p1
                    closest_x = p1[0]
                    closest_y = p1[1]
                elif proj > edge_len:
                    # 最近點就是 p2
                    closest_x = p2[0]
                    closest_y = p2[1]
                else:
                    # 最近點在邊的中間
                    closest_x = p1[0] + exn * proj
                    closest_y = p1[1] + eyn * proj

                # 判斷球心至最近點的距離
                dist = math.hypot(closest_x - ball.x, closest_y - ball.y)
                if dist < ball.radius:
                    # 撞到牆了，計算法向量
                    nx = ball.x - closest_x
                    ny = ball.y - closest_y
                    ndist = math.hypot(nx, ny)
                    if ndist == 0:
                        # 理論上很少發生，多半是球心正好在邊上
                        continue
                    nx /= ndist
                    ny /= ndist
                    # 修正球心位置，避免嵌入牆
                    overlap = ball.radius - dist
                    ball.x += nx * overlap
                    ball.y += ny * overlap
                    # 速度沿法向量反射
                    vn = ball.vx * nx + ball.vy * ny
                    # 加入牆面彈性係數
                    vn_reflect = -RESTITUTION_WALL * vn
                    # 切向速度不變
                    vt_x = ball.vx - vn * nx
                    vt_y = ball.vy - vn * ny
                    # spin 變動 (以切向速度為基礎來做簡單近似)
                    spin_impact = 0.02 * vn  # 可以視需要調整
                    ball.spin += spin_impact

                    # 最後新速度
                    ball.vx = vt_x + vn_reflect * nx
                    ball.vy = vt_y + vn_reflect * ny

    def update_canvas(self):
        # 更新物理
        self.update_physics()

        # 清除畫布
        self.canvas.delete("all")

        # 先畫出七邊形
        hept_points = self.get_heptagon_points(self.heptagon_angle)
        flat_points = []
        for px, py in hept_points:
            flat_points.append(px)
            flat_points.append(py)
        self.canvas.create_polygon(flat_points, fill="", outline="black", width=2)

        # 畫出球
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")
            # 顯示數字與當前自轉 (spin)
            # 這裡不旋轉文字，只在球中心位置顯示
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=f"{ball.number}\nSpin:{ball.spin:.1f}",
                fill="black",
                font=("Arial", 10, "bold"),
            )

        # 設置下一次更新(約 60FPS)
        self.master.after(int(TIME_STEP * 1000), self.update_canvas)


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    app = SpinningHeptagonSimulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
