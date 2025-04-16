"""
heptagon_balls.py
Python 3.8+ / Tk 8.6
依赖：tkinter, numpy
"""

import sys
import math
import time
import tkinter as tk
from dataclasses import dataclass
from typing import List
import numpy as np

# ---------- 物理与场景常量 ----------
N_BALLS = 20
R_BALL = 20.0  # 小球半径 (px)
G = 500.0  # 重力加速度 (px/s^2)
DAMPING = 0.999  # 空气阻尼
RESTI_BB = 0.9  # 球-球恢复系数
RESTI_BW = 0.8  # 球-壁恢复系数
OMEGA_HEP = 2 * math.pi / 5  # 七边形角速度 (rad/s)  -> 360°/5s
FPS = 60
DT = 1.0 / FPS

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

# ---------- 画布尺寸与七边形尺寸 ----------
CANVAS_W = 800
CANVAS_H = 800
CENTER = np.array([CANVAS_W / 2, CANVAS_H / 2])
R_HEP = 300  # 七边形外接圆半径
APOTHEM = R_HEP * math.cos(math.pi / 7)  # 七边形到边的内切半径

# 预计算七边形在“物体坐标系”下的 7 个外法线 (单位向量)
NORMALS_BODY = [
    np.array(
        [math.cos(2 * math.pi * k / 7), math.sin(2 * math.pi * k / 7)], dtype=float
    )
    for k in range(7)
]


@dataclass
class Ball:
    idx: int
    pos: np.ndarray  # 2‑D 位置
    vel: np.ndarray  # 2‑D 速度
    spin: float  # 角速度 (rad/s) – 仅用于显示
    angle: float  # 当前自转角 (rad)
    color: str
    canvas_id: int = None  # 画布 oval id
    text_id: int = None  # 画布 text id


# ---------- 帮助函数 ----------
def rotate_vec(v: np.ndarray, theta: float) -> np.ndarray:
    """逆时针旋转向量 v θ 弧度"""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def project(v: np.ndarray, n: np.ndarray) -> float:
    """v 在 n 方向上的标量投影 (n 单位向量)"""
    return float(np.dot(v, n))


# ---------- 核心模拟类 ----------
class HeptagonSim:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(self.root, width=CANVAS_W, height=CANVAS_H, bg="white")
        self.canvas.pack()

        # 七边形角度
        self.theta = 0.0

        # 创建小球
        self.balls: List[Ball] = []
        for i in range(N_BALLS):
            b = Ball(
                idx=i + 1,
                pos=CENTER.copy(),  # 全部从中心坠落
                vel=np.random.uniform(-50, 50, size=2),  # 初始化轻微随机速度
                spin=np.random.uniform(-4, 4),
                angle=0.0,
                color=COLORS[i],
            )
            self.balls.append(b)
            # 先在 canvas 留占位，后续 update_shapes 会真正调整
            b.canvas_id = self.canvas.create_oval(0, 0, 0, 0, fill=b.color, outline="")
            b.text_id = self.canvas.create_text(
                0, 0, text=str(b.idx), font=("Helvetica", 12, "bold")
            )
        # 七边形图形 id
        self.hept_id = self.canvas.create_polygon([], outline="#555", fill="", width=2)

        # 主循环
        self.prev_time = time.time()
        self.running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(int(1000 / FPS), self.tick)
        self.root.mainloop()

    # ---------- 物理更新 ----------
    def physics_step(self, dt: float):
        # 更新七边形角度
        self.theta = (self.theta + OMEGA_HEP * dt) % (2 * math.pi)
        omega_vec = np.array([0, 0, OMEGA_HEP])  # 仅 z 分量用于计算壁面速度

        # 计算球-球碰撞 (简单 O(n^2))
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.resolve_ball_collision(self.balls[i], self.balls[j], dt)

        # 更新各球
        for b in self.balls:
            # 重力
            b.vel[1] += G * dt
            # 速度阻尼
            b.vel *= DAMPING
            # 位置积分
            b.pos += b.vel * dt
            # 自转更新
            b.angle += b.spin * dt
            b.spin *= 0.995  # 自转阻尼

            # 球-壁碰撞
            self.resolve_wall_collision(b, omega_vec, dt)

    # ---------- 球-球碰撞 ----------
    def resolve_ball_collision(self, a: Ball, b: Ball, dt: float):
        diff = b.pos - a.pos
        dist = np.linalg.norm(diff)
        min_dist = 2 * R_BALL
        if dist == 0:
            # 完全重合，给个随机小扰动
            diff = np.random.uniform(-1, 1, size=2)
            dist = np.linalg.norm(diff)
        if dist < min_dist:
            n = diff / dist
            overlap = min_dist - dist
            # 将两球沿法线分开一半距离
            a.pos -= n * overlap / 2
            b.pos += n * overlap / 2
            # 相对速度
            rel = b.vel - a.vel
            vn = project(rel, n)
            if vn < 0:
                # 动量守恒 + 恢复系数
                imp = -(1 + RESTI_BB) * vn / 2
                a.vel -= imp * n
                b.vel += imp * n
                # 简单角动量耦合：切向速度带来自转
                t = np.array([-n[1], n[0]])
                vt = project(rel, t)
                a.spin -= vt / R_BALL * 0.05
                b.spin += vt / R_BALL * 0.05

    # ---------- 球-壁碰撞 ----------
    def resolve_wall_collision(self, b: Ball, omega_vec: np.ndarray, dt: float):
        # 当前旋转法线集合
        for n_body in NORMALS_BODY:
            n_world = rotate_vec(n_body, self.theta)  # 法线单位向量
            # 球心到中心向量 (以中心为原点)
            local_pos = b.pos - CENTER
            # 球心到边的有符号距离
            dist_to_side = project(local_pos, n_world)
            if dist_to_side > APOTHEM - R_BALL:
                # 穿透深度
                overlap = dist_to_side - (APOTHEM - R_BALL)
                # 将球推回内部
                b.pos -= n_world * overlap
                # 壁面在碰撞点速度 = ω × r
                r_vec = np.append(local_pos, 0)
                wall_v = np.cross(omega_vec, r_vec)[:2]
                # 相对速度
                v_rel = b.vel - wall_v
                vn = project(v_rel, n_world)
                if vn > 0:  # 球朝着外法线移动才需要反射
                    v_rel = v_rel - (1 + RESTI_BW) * vn * n_world
                    # 简单切向摩擦+自转
                    t = np.array([-n_world[1], n_world[0]])
                    vt = project(v_rel, t)
                    v_rel -= 0.2 * vt * t  # 切向速度衰减
                    b.spin += vt / R_BALL * 0.1
                    # 组合回世界速度
                    b.vel = v_rel + wall_v

    # ---------- 绘制更新 ----------
    def update_shapes(self):
        # 更新七边形顶点
        vertices = []
        for k in range(7):
            ang = 2 * math.pi * k / 7 + self.theta
            vx = CENTER[0] + R_HEP * math.cos(ang)
            vy = CENTER[1] + R_HEP * math.sin(ang)
            vertices.extend([vx, vy])
        self.canvas.coords(self.hept_id, *vertices)

        # 更新小球
        for b in self.balls:
            x, y = b.pos
            self.canvas.coords(
                b.canvas_id, x - R_BALL, y - R_BALL, x + R_BALL, y + R_BALL
            )
            # 数字沿圆周旋转显示
            offset = 0.6 * R_BALL * np.array([math.cos(b.angle), math.sin(b.angle)])
            tx, ty = x + offset[0], y + offset[1]
            self.canvas.coords(b.text_id, tx, ty)

    # ---------- 帧循环 ----------
    def tick(self):
        if not self.running:
            return
        now = time.time()
        dt = now - self.prev_time
        # 若帧耗时过大，避免一次性积分太多物理
        dt = min(dt, 0.05)
        self.prev_time = now

        # 物理 & 渲染
        self.physics_step(dt)
        self.update_shapes()

        # 下一帧
        self.root.after(int(1000 / FPS), self.tick)

    def on_close(self):
        self.running = False
        self.root.destroy()


# ---------- 主入口 ----------
if __name__ == "__main__":
    # 兼容 Windows 高 DPI
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    HeptagonSim()
    sys.exit(0)
