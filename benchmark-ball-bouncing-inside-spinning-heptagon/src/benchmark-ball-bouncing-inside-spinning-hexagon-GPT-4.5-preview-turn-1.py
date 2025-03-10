import tkinter as tk
import numpy as np
import math
from dataclasses import dataclass
from typing import List

# 常量定义
WIDTH, HEIGHT = 800, 800
BALL_COUNT = 20
BALL_RADIUS = 15
GRAVITY = 0.3
FRICTION = 0.99
ROTATION_SPEED = 2 * math.pi / (5 * 60)  # 每帧旋转角度 (360度/5秒，假设60fps)
HEPTAGON_RADIUS = 300
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
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
    pos: np.ndarray
    vel: np.ndarray
    color: str
    number: int
    spin: float  # 球的旋转角度
    spin_speed: float  # 球的旋转速度


class Simulation:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.angle = 0
        self.balls: List[Ball] = []
        self.init_balls()
        self.animate()

    def init_balls(self):
        for i in range(BALL_COUNT):
            ball = Ball(
                pos=CENTER.copy(),
                vel=np.random.uniform(-2, 2, size=2),
                color=COLORS[i],
                number=i + 1,
                spin=0,
                spin_speed=np.random.uniform(-0.2, 0.2),
            )
            self.balls.append(ball)

    def rotate_point(self, point, angle):
        c, s = np.cos(angle), np.sin(angle)
        rot_matrix = np.array([[c, -s], [s, c]])
        return CENTER + rot_matrix @ (point - CENTER)

    def get_heptagon_vertices(self):
        vertices = []
        for i in range(7):
            theta = self.angle + i * 2 * math.pi / 7
            x = CENTER[0] + HEPTAGON_RADIUS * math.cos(theta)
            y = CENTER[1] + HEPTAGON_RADIUS * math.sin(theta)
            vertices.append(np.array([x, y]))
        return vertices

    def ball_wall_collision(self, ball, vertices):
        for i in range(len(vertices)):
            p1, p2 = vertices[i], vertices[(i + 1) % len(vertices)]
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
            normal /= np.linalg.norm(normal)
            to_ball = ball.pos - p1
            dist = np.dot(to_ball, normal)
            if dist < BALL_RADIUS:
                ball.pos += normal * (BALL_RADIUS - dist)
                ball.vel -= 2 * np.dot(ball.vel, normal) * normal
                ball.vel *= FRICTION
                ball.spin_speed *= FRICTION

    def ball_ball_collision(self, ball1, ball2):
        delta = ball2.pos - ball1.pos
        dist = np.linalg.norm(delta)
        if dist < 2 * BALL_RADIUS and dist != 0:
            overlap = 2 * BALL_RADIUS - dist
            direction = delta / dist
            ball1.pos -= direction * overlap / 2
            ball2.pos += direction * overlap / 2
            v1, v2 = ball1.vel, ball2.vel
            ball1.vel = v1 - np.dot(v1 - v2, direction) * direction
            ball2.vel = v2 - np.dot(v2 - v1, direction) * direction
            ball1.vel *= FRICTION
            ball2.vel *= FRICTION
            ball1.spin_speed *= FRICTION
            ball2.spin_speed *= FRICTION

    def update(self):
        vertices = self.get_heptagon_vertices()
        for ball in self.balls:
            ball.vel[1] += GRAVITY
            ball.pos += ball.vel
            ball.spin += ball.spin_speed
            self.ball_wall_collision(ball, vertices)
        for i in range(BALL_COUNT):
            for j in range(i + 1, BALL_COUNT):
                self.ball_ball_collision(self.balls[i], self.balls[j])
        self.angle += ROTATION_SPEED

    def draw(self):
        self.canvas.delete("all")
        vertices = self.get_heptagon_vertices()
        coords = [coord for vertex in vertices for coord in vertex]
        self.canvas.create_polygon(coords, outline="black", fill="", width=2)
        for ball in self.balls:
            x, y = ball.pos
            self.canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
            )
            # 绘制数字并旋转表示球的旋转
            angle_deg = math.degrees(ball.spin)
            self.canvas.create_text(
                x, y, text=str(ball.number), angle=angle_deg, fill="white"
            )

    def animate(self):
        self.update()
        self.draw()
        self.root.after(16, self.animate)  # 大约60fps

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    sim = Simulation()
    sim.run()
