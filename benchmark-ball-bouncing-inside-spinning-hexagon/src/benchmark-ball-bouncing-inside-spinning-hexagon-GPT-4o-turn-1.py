import tkinter as tk
import numpy as np
import math
from dataclasses import dataclass
from typing import List

# Constants
NUM_BALLS = 20
BALL_RADIUS = 10
HEPTAGON_RADIUS = 200
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
    x: float
    y: float
    vx: float
    vy: float
    number: int
    color: str
    spin: float = 0.0

    def update(self):
        self.vy += GRAVITY
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.x += self.vx
        self.y += self.vy

    def draw(self, canvas):
        x0 = self.x - BALL_RADIUS
        y0 = self.y - BALL_RADIUS
        x1 = self.x + BALL_RADIUS
        y1 = self.y + BALL_RADIUS
        canvas.create_oval(x0, y0, x1, y1, fill=self.color)
        canvas.create_text(self.x, self.y, text=str(self.number), fill="black")


class Heptagon:
    def __init__(self, radius, spin_speed):
        self.radius = radius
        self.spin_speed = spin_speed
        self.angle = 0

    def update(self):
        self.angle += self.spin_speed / 60  # assuming 60 FPS
        self.angle %= 360

    def get_vertices(self):
        vertices = []
        for i in range(7):
            angle_deg = self.angle + i * 360 / 7
            angle_rad = math.radians(angle_deg)
            x = self.radius * math.cos(angle_rad)
            y = self.radius * math.sin(angle_rad)
            vertices.append((x, y))
        return vertices

    def draw(self, canvas, center_x, center_y):
        vertices = self.get_vertices()
        points = []
        for x, y in vertices:
            points.extend([center_x + x, center_y + y])
        canvas.create_polygon(points, outline="black", fill="", width=2)


def check_collision(ball, heptagon):
    # Check collision with heptagon walls
    vertices = heptagon.get_vertices()
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]
        # Line equation: Ax + By + C = 0
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2
        dist = abs(A * ball.x + B * ball.y + C) / math.sqrt(A**2 + B**2)
        if dist < BALL_RADIUS:
            # Reflect the ball
            normal = np.array([A, B])
            normal = normal / np.linalg.norm(normal)
            velocity = np.array([ball.vx, ball.vy])
            ball.vx, ball.vy = velocity - 2 * np.dot(velocity, normal) * normal
            ball.spin += np.linalg.norm(velocity) * 0.1  # Add spin


def check_ball_collisions(balls):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            dx = balls[j].x - balls[i].x
            dy = balls[j].y - balls[i].y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < 2 * BALL_RADIUS:
                # Simple elastic collision
                nx, ny = dx / distance, dy / distance
                p = (
                    2
                    * (
                        balls[i].vx * nx
                        + balls[i].vy * ny
                        - balls[j].vx * nx
                        - balls[j].vy * ny
                    )
                    / 2
                )
                balls[i].vx -= p * nx
                balls[i].vy -= p * ny
                balls[j].vx += p * nx
                balls[j].vy += p * ny


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=600, height=600, bg="white")
    canvas.pack()

    heptagon = Heptagon(HEPTAGON_RADIUS, SPIN_SPEED)
    balls = [
        Ball(
            300,
            300,
            np.random.uniform(-2, 2),
            np.random.uniform(-2, 2),
            i + 1,
            COLORS[i],
        )
        for i in range(NUM_BALLS)
    ]

    def update():
        canvas.delete("all")
        heptagon.update()
        heptagon.draw(canvas, 300, 300)
        for ball in balls:
            ball.update()
            check_collision(ball, heptagon)
            ball.draw(canvas)
        check_ball_collisions(balls)
        root.after(16, update)  # roughly 60 FPS

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
