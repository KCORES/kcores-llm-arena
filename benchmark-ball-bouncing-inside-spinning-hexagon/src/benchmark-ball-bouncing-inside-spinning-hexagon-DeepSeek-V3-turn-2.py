import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 360 / 5  # degrees per second
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
    id: int
    x: float
    y: float
    vx: float
    vy: float
    angle: float
    spin: float


def rotate_point(x, y, angle):
    rad = math.radians(angle)
    return x * math.cos(rad) - y * math.sin(rad), x * math.sin(rad) + y * math.cos(rad)


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def collide_balls(ball1, ball2):
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    if dist < 2 * BALL_RADIUS:
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        angle = math.atan2(dy, dx)
        speed1 = math.sqrt(ball1.vx**2 + ball1.vy**2)
        speed2 = math.sqrt(ball2.vx**2 + ball2.vy**2)
        ball1.vx = speed2 * math.cos(angle)
        ball1.vy = speed2 * math.sin(angle)
        ball2.vx = speed1 * math.cos(angle + math.pi)
        ball2.vy = speed1 * math.sin(angle + math.pi)


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=800, height=800)
        self.canvas.pack()
        self.balls = [Ball(i, 400, 400, 0, 0, 0, 0) for i in range(20)]
        self.angle = 0
        self.last_time = 0
        self.root.after(0, self.update)

    def draw_heptagon(self):
        points = []
        for i in range(7):
            x = 400 + HEPTAGON_RADIUS * math.cos(math.radians(self.angle + i * 360 / 7))
            y = 400 + HEPTAGON_RADIUS * math.sin(math.radians(self.angle + i * 360 / 7))
            points.extend([x, y])
        self.canvas.create_polygon(points, outline="black", fill="")

    def draw_balls(self):
        for ball in self.balls:
            x, y = rotate_point(ball.x - 400, ball.y - 400, -self.angle)
            x += 400
            y += 400
            self.canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=COLORS[ball.id],
            )
            self.canvas.create_text(x, y, text=str(ball.id + 1))

    def update(self):
        self.canvas.delete("all")
        current_time = self.root.after_info()[0]
        dt = (current_time - self.last_time) / 1000
        self.last_time = current_time

        self.angle += SPIN_SPEED * dt
        if self.angle >= 360:
            self.angle -= 360

        for ball in self.balls:
            ball.vy += GRAVITY
            ball.x += ball.vx
            ball.y += ball.vy
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.angle += ball.spin
            ball.spin *= FRICTION

            dist = distance(ball.x, ball.y, 400, 400)
            if dist + BALL_RADIUS > HEPTAGON_RADIUS:
                angle = math.atan2(ball.y - 400, ball.x - 400)
                ball.x = 400 + (HEPTAGON_RADIUS - BALL_RADIUS) * math.cos(angle)
                ball.y = 400 + (HEPTAGON_RADIUS - BALL_RADIUS) * math.sin(angle)
                ball.vx = -ball.vx * 0.8
                ball.vy = -ball.vy * 0.8

        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                collide_balls(self.balls[i], self.balls[j])

        self.draw_heptagon()
        self.draw_balls()
        self.root.after(16, self.update)


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
