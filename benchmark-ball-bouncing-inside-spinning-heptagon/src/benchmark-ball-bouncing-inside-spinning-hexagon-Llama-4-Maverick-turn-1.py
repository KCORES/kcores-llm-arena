import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
BALL_RADIUS = 15
HEPTAGON_SIZE = 300
FPS = 60
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 360 / 5  # degrees per second
COLLISION_DAMPENING = 0.7  # between 0 and 1

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
    color: str
    number: int

    def update(self):
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.spin += np.random.uniform(-1, 1)


class Heptagon:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.angle = 0

    def get_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.angle + i * math.pi * 2 / 7
            vertices.append(
                (
                    self.x + math.cos(angle) * self.size,
                    self.y + math.sin(angle) * self.size,
                )
            )
        return vertices

    def update(self):
        self.angle += SPIN_SPEED / FPS * math.pi / 180


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def collide_ball_heptagon(ball, heptagon):
    vertices = heptagon.get_vertices()
    for i in range(7):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % 7]
        distance_to_line = abs(
            (p2[0] - p1[0]) * (p1[1] - ball.y) - (p1[0] - ball.x) * (p2[1] - p1[1])
        ) / math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        if distance_to_line < BALL_RADIUS:
            normal = (-(p2[1] - p1[1]), p2[0] - p1[0])
            normal = (
                normal[0] / math.sqrt(normal[0] ** 2 + normal[1] ** 2),
                normal[1] / math.sqrt(normal[0] ** 2 + normal[1] ** 2),
            )
            dot_product = ball.vx * normal[0] + ball.vy * normal[1]
            ball.vx -= (1 + COLLISION_DAMPENING) * dot_product * normal[0]
            ball.vy -= (1 + COLLISION_DAMPENING) * dot_product * normal[1]
            return


def collide_balls(ball1, ball2):
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    distance_between_balls = math.sqrt(dx**2 + dy**2)
    if distance_between_balls < 2 * BALL_RADIUS:
        normal = (dx / distance_between_balls, dy / distance_between_balls)
        tangent = (-normal[1], normal[0])
        v1n = ball1.vx * normal[0] + ball1.vy * normal[1]
        v1t = ball1.vx * tangent[0] + ball1.vy * tangent[1]
        v2n = ball2.vx * normal[0] + ball2.vy * normal[1]
        v2t = ball2.vx * tangent[0] + ball2.vy * tangent[1]
        v1n, v2n = v2n * COLLISION_DAMPENING, v1n * COLLISION_DAMPENING
        ball1.vx = v1n * normal[0] + v1t * tangent[0]
        ball1.vy = v1n * normal[1] + v1t * tangent[1]
        ball2.vx = v2n * normal[0] + v2t * tangent[0]
        ball2.vy = v2n * normal[1] + v2t * tangent[1]


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
    canvas.pack()

    heptagon = Heptagon(WIDTH // 2, HEIGHT // 2, HEPTAGON_SIZE)
    balls: List[Ball] = []
    for i in range(20):
        ball = Ball(
            WIDTH // 2,
            HEIGHT // 2,
            np.random.uniform(-2, 2),
            np.random.uniform(-2, 2),
            0,
            BALL_COLORS[i],
            i + 1,
        )
        balls.append(ball)

    def update():
        heptagon.update()
        for ball in balls:
            ball.update()
            collide_ball_heptagon(ball, heptagon)
            for other_ball in balls:
                if ball != other_ball:
                    collide_balls(ball, other_ball)

        canvas.delete("all")
        vertices = heptagon.get_vertices()
        canvas.create_polygon(vertices, outline="black")
        for ball in balls:
            x = ball.x
            y = ball.y
            canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
            )
            canvas.create_text(x, y, text=str(ball.number), font=("Arial", 12))
            canvas.create_line(
                x,
                y,
                x + math.cos(ball.spin) * BALL_RADIUS,
                y + math.sin(ball.spin) * BALL_RADIUS,
                fill="white",
            )

        root.after(int(1000 / FPS), update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
