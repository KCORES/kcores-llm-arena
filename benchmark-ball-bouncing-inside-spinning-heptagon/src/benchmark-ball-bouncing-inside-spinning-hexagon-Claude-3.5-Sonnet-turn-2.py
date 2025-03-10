import tkinter as tk
from dataclasses import dataclass
from typing import List, Tuple
import math
import numpy as np
from time import time


@dataclass
class Ball:
    x: float  # 中心x坐标
    y: float  # 中心y坐标
    vx: float  # x方向速度
    vy: float  # y方向速度
    angle: float  # 旋转角度
    angular_velocity: float  # 角速度
    number: int  # 球上的数字
    color: str  # 球的颜色


class BouncingBallsApp:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Bouncing Balls in Spinning Heptagon")

        # 常量定义
        self.WIDTH = 800
        self.HEIGHT = 800
        self.BALL_RADIUS = 20
        self.HEPTAGON_RADIUS = 300
        self.GRAVITY = 980.0  # 像素/秒²
        self.FRICTION = 0.8  # 碰撞能量损失系数
        self.ANGULAR_FRICTION = 0.95  # 旋转摩擦系数

        # 创建画布
        self.canvas = tk.Canvas(self.window, width=self.WIDTH, height=self.HEIGHT)
        self.canvas.pack()

        # 初始化小球
        self.colors = [
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

        self.balls = []
        self.init_balls()

        # 七边形旋转角度
        self.heptagon_angle = 0
        self.last_time = time()

        # 开始动画
        self.update()
        self.window.mainloop()

    def init_balls(self):
        center_x = self.WIDTH / 2
        center_y = self.HEIGHT / 2
        for i in range(20):
            # 给每个球一个随机的初始速度
            angle = 2 * math.pi * i / 20
            speed = 100
            self.balls.append(
                Ball(
                    x=center_x,
                    y=center_y,
                    vx=speed * math.cos(angle),
                    vy=speed * math.sin(angle),
                    angle=0,
                    angular_velocity=0,
                    number=i + 1,
                    color=self.colors[i],
                )
            )

    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        center_x = self.WIDTH / 2
        center_y = self.HEIGHT / 2
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.heptagon_angle
            x = center_x + self.HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + self.HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def check_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        for i in range(7):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % 7]

            # 计算球到线段的距离
            wall_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
            wall_length = np.linalg.norm(wall_vector)
            wall_unit = wall_vector / wall_length

            ball_to_p1 = np.array([ball.x - p1[0], ball.y - p1[1]])
            projection = np.dot(ball_to_p1, wall_unit)

            if 0 <= projection <= wall_length:
                normal = np.array([-wall_unit[1], wall_unit[0]])
                distance = abs(np.dot(ball_to_p1, normal))

                if distance < self.BALL_RADIUS:
                    # 碰撞响应
                    velocity = np.array([ball.vx, ball.vy])
                    normal_component = np.dot(velocity, normal) * normal
                    tangent_component = velocity - normal_component

                    # 更新速度和角速度
                    ball.vx, ball.vy = (
                        -normal_component * self.FRICTION
                        + tangent_component * self.FRICTION
                    )
                    ball.angular_velocity = (
                        ball.angular_velocity * self.ANGULAR_FRICTION
                        + np.dot(tangent_component, wall_unit) / self.BALL_RADIUS
                    )

                    # 调整位置防止穿墙
                    correction = (self.BALL_RADIUS - distance) * normal
                    ball.x += correction[0]
                    ball.y += correction[1]

    def check_ball_collision(self, b1: Ball, b2: Ball):
        dx = b2.x - b1.x
        dy = b2.y - b1.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 2 * self.BALL_RADIUS:
            # 碰撞响应
            normal = np.array([dx / distance, dy / distance])

            v1 = np.array([b1.vx, b1.vy])
            v2 = np.array([b2.vx, b2.vy])

            # 计算碰撞后的速度
            v1_new = v1 - np.dot(v1 - v2, normal) * normal * self.FRICTION
            v2_new = v2 - np.dot(v2 - v1, -normal) * normal * self.FRICTION

            b1.vx, b1.vy = v1_new
            b2.vx, b2.vy = v2_new

            # 更新角速度
            relative_velocity = v2 - v1
            b1.angular_velocity = (
                b1.angular_velocity * self.ANGULAR_FRICTION
                + np.dot(relative_velocity, [-normal[1], normal[0]]) / self.BALL_RADIUS
            )
            b2.angular_velocity = (
                b2.angular_velocity * self.ANGULAR_FRICTION
                - np.dot(relative_velocity, [-normal[1], normal[0]]) / self.BALL_RADIUS
            )

            # 调整位置防止重叠
            overlap = 2 * self.BALL_RADIUS - distance
            correction = overlap * normal / 2
            b1.x -= correction[0]
            b1.y -= correction[1]
            b2.x += correction[0]
            b2.y += correction[1]

    def update(self):
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 更新七边形旋转
        self.heptagon_angle += 2 * math.pi * dt / 5  # 5秒转一圈

        # 更新所有球的位置和速度
        for ball in self.balls:
            ball.vy += self.GRAVITY * dt
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            ball.angle += ball.angular_velocity * dt

        # 检查碰撞
        vertices = self.get_heptagon_vertices()
        for ball in self.balls:
            self.check_wall_collision(ball, vertices)

        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.check_ball_collision(self.balls[i], self.balls[j])

        # 重绘
        self.canvas.delete("all")

        # 绘制七边形
        self.canvas.create_polygon(
            *[coord for vertex in vertices for coord in vertex],
            outline="black",
            fill="white"
        )

        # 绘制球
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - self.BALL_RADIUS,
                ball.y - self.BALL_RADIUS,
                ball.x + self.BALL_RADIUS,
                ball.y + self.BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )

            # 绘制数字（考虑旋转）
            text_x = ball.x + self.BALL_RADIUS * 0.5 * math.cos(ball.angle)
            text_y = ball.y + self.BALL_RADIUS * 0.5 * math.sin(ball.angle)
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                font=("Arial", int(self.BALL_RADIUS * 0.8)),
            )

        self.window.after(16, self.update)  # 约60 FPS


if __name__ == "__main__":
    app = BouncingBallsApp()
