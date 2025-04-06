import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 600
BALL_RADIUS = 10
HEPTAGON_SIZE = 200
GRAVITY = 0.1
FRICTION = 0.99
BOUNCE_DAMPENING = 0.7
ROTATION_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds

# Colors
BALL_COLORS = [
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
    spin: float
    number: int
    color: str

    def update(self):
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.spin *= FRICTION

    def bounce(self, normal_x, normal_y):
        dot_product = self.vx * normal_x + self.vy * normal_y
        self.vx -= (1 + BOUNCE_DAMPENING) * dot_product * normal_x
        self.vy -= (1 + BOUNCE_DAMPENING) * dot_product * normal_y


class Heptagon:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.angle = 0
        self.vertices = self.calculate_vertices()

    def calculate_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.angle + 2 * math.pi * i / 7
            vertex_x = self.x + self.size * math.cos(angle)
            vertex_y = self.y + self.size * math.sin(angle)
            vertices.append((vertex_x, vertex_y))
        return vertices

    def update(self):
        self.angle += ROTATION_SPEED / 60  # Assuming 60 FPS
        self.vertices = self.calculate_vertices()


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def check_collision(ball, heptagon):
    for i in range(7):
        x1, y1 = heptagon.vertices[i]
        x2, y2 = heptagon.vertices[(i + 1) % 7]
        distance_to_line = abs(
            (x2 - x1) * (y1 - ball.y) - (x1 - ball.x) * (y2 - y1)
        ) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if distance_to_line < BALL_RADIUS:
            normal_x = y2 - y1
            normal_y = x1 - x2
            length = math.sqrt(normal_x**2 + normal_y**2)
            normal_x /= length
            normal_y /= length
            ball.bounce(normal_x, normal_y)


def check_ball_collision(ball1, ball2):
    distance_between_balls = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    if distance_between_balls < 2 * BALL_RADIUS:
        normal_x = ball2.x - ball1.x
        normal_y = ball2.y - ball1.y
        length = math.sqrt(normal_x**2 + normal_y**2)
        normal_x /= length
        normal_y /= length
        ball1.bounce(normal_x, normal_y)
        ball2.bounce(-normal_x, -normal_y)


class App:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.heptagon = Heptagon(WIDTH / 2, HEIGHT / 2, HEPTAGON_SIZE)
        self.balls = self.initialize_balls()
        self.update()

    def initialize_balls(self):
        balls = []
        for i in range(20):
            ball = Ball(
                WIDTH / 2,
                HEIGHT / 2,
                np.random.uniform(-2, 2),
                np.random.uniform(-2, 2),
                np.random.uniform(-1, 1),
                i + 1,
                BALL_COLORS[i],
            )
            balls.append(ball)
        return balls

    def update(self):
        self.heptagon.update()
        for ball in self.balls:
            ball.update()
            check_collision(ball, self.heptagon)
            for other_ball in self.balls:
                if ball != other_ball:
                    check_ball_collision(ball, other_ball)
        self.draw()
        self.root.after(16, self.update)  # 60 FPS

    def draw(self):
        self.canvas.delete("all")
        self.canvas.create_polygon(self.heptagon.vertices, outline="black")
        for ball in self.balls:
            x1, y1 = ball.x - BALL_RADIUS, ball.y - BALL_RADIUS
            x2, y2 = ball.x + BALL_RADIUS, ball.y + BALL_RADIUS
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), angle=math.degrees(ball.spin)
            )
        self.root.update()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
