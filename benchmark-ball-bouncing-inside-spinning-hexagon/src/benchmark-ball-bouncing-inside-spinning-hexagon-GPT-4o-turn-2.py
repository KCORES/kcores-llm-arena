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

    def update_position(self):
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.vx *= FRICTION
        self.vy *= FRICTION

    def draw(self, canvas):
        canvas.create_oval(
            self.x - BALL_RADIUS,
            self.y - BALL_RADIUS,
            self.x + BALL_RADIUS,
            self.y + BALL_RADIUS,
            fill=self.color,
        )
        canvas.create_text(self.x, self.y, text=str(self.number), fill="white")


class Heptagon:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.angle = 0

    def update(self, dt):
        self.angle += SPIN_SPEED * dt
        self.angle %= 360

    def get_vertices(self):
        vertices = []
        for i in range(7):
            angle_deg = self.angle + i * 360 / 7
            angle_rad = math.radians(angle_deg)
            x = self.center_x + self.radius * math.cos(angle_rad)
            y = self.center_y + self.radius * math.sin(angle_rad)
            vertices.append((x, y))
        return vertices

    def draw(self, canvas):
        vertices = self.get_vertices()
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % len(vertices)]
            canvas.create_line(x1, y1, x2, y2, fill="black")


class Simulation:
    def __init__(self, canvas):
        self.canvas = canvas
        self.heptagon = Heptagon(300, 300, HEPTAGON_RADIUS)
        self.balls = [
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

    def update(self, dt):
        self.heptagon.update(dt)
        for ball in self.balls:
            ball.update_position()
            self.handle_collisions(ball)

    def handle_collisions(self, ball):
        # Collision with heptagon walls
        vertices = self.heptagon.get_vertices()
        for i in range(len(vertices)):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % len(vertices)]
            if self.line_circle_collision(x1, y1, x2, y2, ball.x, ball.y, BALL_RADIUS):
                # Reflect ball velocity
                normal = np.array([y2 - y1, x1 - x2])
                normal = normal / np.linalg.norm(normal)
                velocity = np.array([ball.vx, ball.vy])
                ball.vx, ball.vy = velocity - 2 * np.dot(velocity, normal) * normal

        # Collision with other balls
        for other in self.balls:
            if other is not ball:
                dx = other.x - ball.x
                dy = other.y - ball.y
                distance = math.sqrt(dx**2 + dy**2)
                if distance < 2 * BALL_RADIUS:
                    # Simple elastic collision response
                    nx, ny = dx / distance, dy / distance
                    p = (
                        2
                        * (ball.vx * nx + ball.vy * ny - other.vx * nx - other.vy * ny)
                        / 2
                    )
                    ball.vx -= p * nx
                    ball.vy -= p * ny
                    other.vx += p * nx
                    other.vy += p * ny

    def line_circle_collision(self, x1, y1, x2, y2, cx, cy, radius):
        # Check if the circle collides with the line segment
        ac = np.array([cx - x1, cy - y1])
        ab = np.array([x2 - x1, y2 - y1])
        ab_norm = ab / np.linalg.norm(ab)
        projection = np.dot(ac, ab_norm)
        closest = np.array([x1, y1]) + projection * ab_norm
        if np.linalg.norm(closest - np.array([x1, y1])) > np.linalg.norm(ab):
            closest = np.array([x2, y2])
        elif np.linalg.norm(closest - np.array([x2, y2])) > np.linalg.norm(ab):
            closest = np.array([x1, y1])
        return np.linalg.norm(closest - np.array([cx, cy])) < radius

    def draw(self):
        self.canvas.delete("all")
        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas)


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=600, height=600, bg="white")
    canvas.pack()

    simulation = Simulation(canvas)

    def update_frame():
        simulation.update(1 / 60)
        simulation.draw()
        root.after(16, update_frame)

    update_frame()
    root.mainloop()


if __name__ == "__main__":
    main()
