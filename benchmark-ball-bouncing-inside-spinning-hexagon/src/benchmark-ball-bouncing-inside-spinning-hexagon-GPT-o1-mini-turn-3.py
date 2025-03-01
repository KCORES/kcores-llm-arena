import tkinter as tk
import math
import random
import time
from dataclasses import dataclass
from typing import List

# Constants
WIDTH = 800
HEIGHT = 600
HEPTAGON_SIDES = 7
HEPTAGON_SIZE = 250
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.99
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
SPIN_SPEED_DEG_PER_SEC = 360 / 5  # 360 degrees per 5 seconds
SPIN_SPEED_RAD_PER_SEC = math.radians(SPIN_SPEED_DEG_PER_SEC)


@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    color: str
    spin: float

    def move(self):
        self.vy += GRAVITY
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.x += self.vx
        self.y += self.vy


@dataclass
class Heptagon:
    center_x: float
    center_y: float
    size: float
    angle: float = 0  # Current rotation angle in radians

    def get_vertices(self) -> List[tuple]:
        vertices = []
        angle_between = 2 * math.pi / HEPTAGON_SIDES
        for i in range(HEPTAGON_SIDES):
            theta = self.angle + i * angle_between
            x = self.center_x + self.size * math.cos(theta)
            y = self.center_y + self.size * math.sin(theta)
            vertices.append((x, y))
        return vertices

    def rotate(self, delta_time):
        self.angle += SPIN_SPEED_RAD_PER_SEC * delta_time
        self.angle = self.angle % (2 * math.pi)


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.heptagon = Heptagon(WIDTH / 2, HEIGHT / 2, HEPTAGON_SIZE)
        self.balls: List[Ball] = []
        self.create_balls()
        self.last_time = time.time()
        self.animate()

    def create_balls(self):
        for i in range(20):
            ball = Ball(
                id=i + 1,
                x=self.heptagon.center_x,
                y=self.heptagon.center_y,
                vx=random.uniform(-5, 5),
                vy=random.uniform(-5, 5),
                color=BALL_COLORS[i],
                spin=random.uniform(-5, 5),
            )
            self.balls.append(ball)

    def animate(self):
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        self.heptagon.rotate(delta_time)

        self.canvas.delete("all")
        self.draw_heptagon()
        self.move_balls()
        self.handle_collisions()
        self.draw_balls()

        self.root.after(16, self.animate)  # Approximately 60 FPS

    def draw_heptagon(self):
        vertices = self.heptagon.get_vertices()
        for i in range(len(vertices)):
            start = vertices[i]
            end = vertices[(i + 1) % len(vertices)]
            self.canvas.create_line(
                start[0], start[1], end[0], end[1], fill="black", width=2
            )

    def move_balls(self):
        for ball in self.balls:
            ball.move()
            # Collision with heptagon walls
            self.collide_with_heptagon(ball)

    def collide_with_heptagon(self, ball: Ball):
        vertices = self.heptagon.get_vertices()
        for i in range(len(vertices)):
            start = vertices[i]
            end = vertices[(i + 1) % len(vertices)]
            # Line segment from start to end
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length == 0:
                continue
            nx = -dy / length
            ny = dx / length
            # Vector from start to ball
            vx = ball.x - start[0]
            vy = ball.y - start[1]
            # Distance from ball to line
            distance = vx * nx + vy * ny
            if distance < BALL_RADIUS:
                # Reflect velocity
                ball.vx -= 2 * (ball.vx * nx + ball.vy * ny) * nx
                ball.vy -= 2 * (ball.vx * nx + ball.vy * ny) * ny
                # Move ball out of the wall
                ball.x += nx * (BALL_RADIUS - distance)
                ball.y += ny * (BALL_RADIUS - distance)

    def handle_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist = math.hypot(dx, dy)
                if dist < 2 * BALL_RADIUS:
                    # Simple elastic collision
                    angle = math.atan2(dy, dx)
                    total_vx = b1.vx - b2.vx
                    total_vy = b1.vy - b2.vy
                    if total_vx * math.cos(angle) + total_vy * math.sin(angle) < 0:
                        # Swap velocities
                        b1.vx, b2.vx = b2.vx, b1.vx
                        b1.vy, b2.vy = b2.vy, b1.vy
                        # Move balls apart
                        overlap = 2 * BALL_RADIUS - dist
                        b1.x -= math.cos(angle) * overlap / 2
                        b1.y -= math.sin(angle) * overlap / 2
                        b2.x += math.cos(angle) * overlap / 2
                        b2.y += math.sin(angle) * overlap / 2

    def draw_balls(self):
        for ball in self.balls:
            x0 = ball.x - BALL_RADIUS
            y0 = ball.y - BALL_RADIUS
            x1 = ball.x + BALL_RADIUS
            y1 = ball.y + BALL_RADIUS
            self.canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")
            self.canvas.create_text(ball.x, ball.y, text=str(ball.id), fill="white")


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
