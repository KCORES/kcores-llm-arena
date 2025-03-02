import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 15
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (60 FPS)
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


# Data classes
@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    angle: float
    spin: float


@dataclass
class Heptagon:
    center: Tuple[float, float]
    radius: float
    angle: float


# Helper functions
def rotate_point(x, y, angle):
    rad = math.radians(angle)
    return (
        x * math.cos(rad) - y * math.sin(rad),
        x * math.sin(rad) + y * math.cos(rad),
    )


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def collide_balls(ball1, ball2):
    dist = distance((ball1.x, ball1.y), (ball2.x, ball2.y))
    if dist < 2 * BALL_RADIUS:
        overlap = 2 * BALL_RADIUS - dist
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        angle = math.atan2(dy, dx)
        ball1.x -= overlap * math.cos(angle) / 2
        ball1.y -= overlap * math.sin(angle) / 2
        ball2.x += overlap * math.cos(angle) / 2
        ball2.y += overlap * math.sin(angle) / 2

        # Swap velocities (simple elastic collision)
        ball1.vx, ball2.vx = ball2.vx, ball1.vx
        ball1.vy, ball2.vy = ball2.vy, ball1.vy


def collide_with_wall(ball, heptagon):
    # Rotate ball position relative to heptagon's rotation
    x, y = rotate_point(
        ball.x - heptagon.center[0], ball.y - heptagon.center[1], -heptagon.angle
    )
    dist = math.sqrt(x**2 + y**2)
    if dist + BALL_RADIUS > heptagon.radius:
        overlap = dist + BALL_RADIUS - heptagon.radius
        angle = math.atan2(y, x)
        ball.x -= overlap * math.cos(angle)
        ball.y -= overlap * math.sin(angle)

        # Reflect velocity
        vx, vy = rotate_point(ball.vx, ball.vy, -heptagon.angle)
        vy *= -1  # Reverse vertical velocity
        ball.vx, ball.vy = rotate_point(vx, vy, heptagon.angle)
        ball.vx *= FRICTION
        ball.vy *= FRICTION


# Main program
class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.heptagon = Heptagon((WIDTH / 2, HEIGHT / 2), HEPTAGON_RADIUS, 0)
        self.balls = self.create_balls()
        self.update()

    def create_balls(self):
        balls = []
        for i in range(20):
            angle = np.random.uniform(0, 2 * math.pi)
            x = self.heptagon.center[0] + np.random.uniform(-50, 50)
            y = self.heptagon.center[1] + np.random.uniform(-50, 50)
            vx = np.random.uniform(-2, 2)
            vy = np.random.uniform(-2, 2)
            spin = np.random.uniform(-5, 5)
            balls.append(Ball(i + 1, x, y, vx, vy, 0, spin))
        return balls

    def update(self):
        self.canvas.delete("all")
        self.heptagon.angle += SPIN_SPEED
        self.draw_heptagon()
        self.update_balls()
        self.root.after(16, self.update)  # ~60 FPS

    def draw_heptagon(self):
        points = []
        for i in range(7):
            angle = math.radians(360 / 7 * i + self.heptagon.angle)
            x = self.heptagon.center[0] + self.heptagon.radius * math.cos(angle)
            y = self.heptagon.center[1] + self.heptagon.radius * math.sin(angle)
            points.extend([x, y])
        self.canvas.create_polygon(points, outline="black", fill="")

    def update_balls(self):
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Update spin
            ball.angle += ball.spin

            # Collide with walls
            collide_with_wall(ball, self.heptagon)

            # Collide with other balls
            for other in self.balls:
                if ball.id != other.id:
                    collide_balls(ball, other)

            # Draw ball
            x1, y1 = ball.x - BALL_RADIUS, ball.y - BALL_RADIUS
            x2, y2 = ball.x + BALL_RADIUS, ball.y + BALL_RADIUS
            self.canvas.create_oval(
                x1, y1, x2, y2, fill=COLORS[ball.id - 1], outline="black"
            )
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.id), font=("Arial", 10)
            )


# Run the program
if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
