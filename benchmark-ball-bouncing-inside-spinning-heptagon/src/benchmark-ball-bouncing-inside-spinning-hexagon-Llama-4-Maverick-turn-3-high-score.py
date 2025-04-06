import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
BALL_RADIUS = 15
HEPTAGON_SIZE = 300
HEPTAGON_SIDES = 7
GRAVITY = 0.1
FRICTION = 0.99
BALL_MATERIAL_BOUNCE_DAMPENING = 0.7
BALL_SPIN_FRICTION = 0.99
HEPTAGON_SPIN_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds

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
    vx: float = 0
    vy: float = 0
    spin: float = 0
    color: str = ""

    def update(self):
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.spin *= BALL_SPIN_FRICTION

    def bounce(self, normal_x, normal_y):
        dot_product = self.vx * normal_x + self.vy * normal_y
        self.vx = (
            self.vx - (1 + BALL_MATERIAL_BOUNCE_DAMPENING) * dot_product * normal_x
        )
        self.vy = (
            self.vy - (1 + BALL_MATERIAL_BOUNCE_DAMPENING) * dot_product * normal_y
        )


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def circle_collision(ball1, ball2):
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    distance_between_centers = math.sqrt(dx**2 + dy**2)
    if distance_between_centers < 2 * BALL_RADIUS:
        normal_x = dx / distance_between_centers
        normal_y = dy / distance_between_centers
        tangent_x = -normal_y
        tangent_y = normal_x

        v1n = ball1.vx * normal_x + ball1.vy * normal_y
        v1t = ball1.vx * tangent_x + ball1.vy * tangent_y

        v2n = ball2.vx * normal_x + ball2.vy * normal_y
        v2t = ball2.vx * tangent_x + ball2.vy * tangent_y

        v1n, v2n = v2n, v1n  # Swap normal velocities

        ball1.vx = v1n * normal_x + v1t * tangent_x
        ball1.vy = v1n * normal_y + v1t * tangent_y

        ball2.vx = v2n * normal_x + v2t * tangent_x
        ball2.vy = v2n * normal_y + v2t * tangent_y


def heptagon_point(angle, size, center_x, center_y):
    return (center_x + size * math.cos(angle), center_y + size * math.sin(angle))


class App:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT)
        self.canvas.pack()

        self.heptagon_angle = 0
        self.balls = []
        for i in range(20):
            ball = Ball(
                WIDTH / 2,
                HEIGHT / 2,
                np.random.uniform(-2, 2),
                np.random.uniform(-2, 2),
                np.random.uniform(-1, 1),
                BALL_COLORS[i],
            )
            ball.spin = np.random.uniform(-10, 10)
            self.balls.append(ball)

        self.animate()

    def animate(self):
        self.canvas.delete("all")

        self.heptagon_angle += HEPTAGON_SPIN_SPEED / 60  # Assuming 60 FPS
        heptagon_points = []
        for i in range(HEPTAGON_SIDES):
            angle = 2 * math.pi * i / HEPTAGON_SIDES + self.heptagon_angle
            x, y = heptagon_point(angle, HEPTAGON_SIZE, WIDTH / 2, HEIGHT / 2)
            heptagon_points.append(x)
            heptagon_points.append(y)

        self.canvas.create_polygon(heptagon_points, outline="black")

        for i, ball in enumerate(self.balls):
            ball.update()

            # Check collision with heptagon walls
            for j in range(HEPTAGON_SIDES):
                x1, y1 = heptagon_point(
                    2 * math.pi * j / HEPTAGON_SIDES + self.heptagon_angle,
                    HEPTAGON_SIZE,
                    WIDTH / 2,
                    HEIGHT / 2,
                )
                x2, y2 = heptagon_point(
                    2 * math.pi * ((j + 1) % HEPTAGON_SIDES) / HEPTAGON_SIDES
                    + self.heptagon_angle,
                    HEPTAGON_SIZE,
                    WIDTH / 2,
                    HEIGHT / 2,
                )
                distance_to_line = abs(
                    (x2 - x1) * (y1 - ball.y) - (x1 - ball.x) * (y2 - y1)
                ) / math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if distance_to_line < BALL_RADIUS:
                    normal_x = y2 - y1
                    normal_y = -(x2 - x1)
                    length = math.sqrt(normal_x**2 + normal_y**2)
                    normal_x /= length
                    normal_y /= length
                    ball.bounce(normal_x, normal_y)

            # Check collision with other balls
            for other_ball in self.balls[i + 1 :]:
                circle_collision(ball, other_ball)

            # Draw ball
            x1, y1 = ball.x - BALL_RADIUS, ball.y - BALL_RADIUS
            x2, y2 = ball.x + BALL_RADIUS, ball.y + BALL_RADIUS
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)
            self.canvas.create_text(ball.x, ball.y, text=str(i + 1), angle=ball.spin)

        self.root.after(16, self.animate)  # 60 FPS


root = tk.Tk()
app = App(root)
root.mainloop()
