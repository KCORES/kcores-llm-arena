import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 350
BALL_RADIUS = 15
GRAVITY = 0.2
FRICTION = 0.99
SPIN_SPEED = 360 / 5  # Degrees per second
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
    id: int
    x: float
    y: float
    vx: float
    vy: float
    color: str
    spin: float


def rotate_point(cx, cy, x, y, angle):
    """Rotate a point around another point by a given angle in degrees."""
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    dx, dy = x - cx, y - cy
    return cx + dx * cos_a - dy * sin_a, cy + dx * sin_a + dy * cos_a


def distance(x1, y1, x2, y2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def line_circle_intersection(px, py, vx, vy, cx, cy, radius):
    """Check if a moving circle intersects with a line segment."""
    a = vx**2 + vy**2
    b = 2 * (vx * (px - cx) + vy * (py - cy))
    c = (px - cx) ** 2 + (py - cy) ** 2 - radius**2
    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        return False, None
    sqrt_discriminant = math.sqrt(discriminant)
    t1 = (-b + sqrt_discriminant) / (2 * a)
    t2 = (-b - sqrt_discriminant) / (2 * a)

    if 0 <= t1 <= 1 or 0 <= t2 <= 1:
        return True, min(t1, t2) if t1 >= 0 and t2 >= 0 else max(t1, t2)
    return False, None


class Simulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.heptagon_angle = 0
        self.balls = []
        self.init_balls()
        self.update()

    def init_balls(self):
        """Initialize 20 balls at the center of the heptagon."""
        for i in range(20):
            ball = Ball(
                id=i + 1,
                x=WIDTH // 2,
                y=HEIGHT // 2,
                vx=np.random.uniform(-2, 2),
                vy=np.random.uniform(-2, 2),
                color=BALL_COLORS[i],
                spin=np.random.uniform(-0.1, 0.1),
            )
            self.balls.append(ball)

    def update(self):
        """Update the simulation state."""
        self.canvas.delete("all")

        # Update heptagon rotation
        self.heptagon_angle = (self.heptagon_angle + SPIN_SPEED / 60) % 360
        self.draw_heptagon()

        # Update balls
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Apply friction
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Handle wall collisions
            self.handle_wall_collision(ball)

            # Handle ball-ball collisions
            self.handle_ball_collisions(ball)

            # Draw ball
            self.draw_ball(ball)

        # Schedule next update
        self.root.after(16, self.update)

    def draw_heptagon(self):
        """Draw the rotating heptagon."""
        cx, cy = WIDTH // 2, HEIGHT // 2
        vertices = []
        for i in range(7):
            angle = math.radians(i * 360 / 7 + self.heptagon_angle)
            x = cx + HEPTAGON_RADIUS * math.cos(angle)
            y = cy + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))

        # Draw edges
        for i in range(7):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 7]
            self.canvas.create_line(x1, y1, x2, y2, fill="white", width=2)

    def draw_ball(self, ball):
        """Draw a ball on the canvas."""
        x, y = ball.x, ball.y
        self.canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=ball.color,
            outline="white",
        )
        self.canvas.create_text(
            x, y, text=str(ball.id), fill="black", font=("Arial", 10)
        )

    def handle_wall_collision(self, ball):
        """Handle collisions with the heptagon walls."""
        cx, cy = WIDTH // 2, HEIGHT // 2
        for i in range(7):
            angle = math.radians(i * 360 / 7 + self.heptagon_angle)
            x1, y1 = rotate_point(
                cx,
                cy,
                cx + HEPTAGON_RADIUS * math.cos(angle),
                cy + HEPTAGON_RADIUS * math.sin(angle),
                -self.heptagon_angle,
            )
            x2, y2 = rotate_point(
                cx,
                cy,
                cx + HEPTAGON_RADIUS * math.cos(angle + math.radians(360 / 7)),
                cy + HEPTAGON_RADIUS * math.sin(angle + math.radians(360 / 7)),
                -self.heptagon_angle,
            )

            # Check collision
            hit, t = line_circle_intersection(
                ball.x, ball.y, ball.vx, ball.vy, x1, y1, BALL_RADIUS
            )
            if hit:
                # Reflect velocity
                nx, ny = y2 - y1, -(x2 - x1)
                norm = math.sqrt(nx**2 + ny**2)
                nx, ny = nx / norm, ny / norm
                dot = ball.vx * nx + ball.vy * ny
                ball.vx -= 2 * dot * nx
                ball.vy -= 2 * dot * ny

    def handle_ball_collisions(self, ball):
        """Handle collisions between balls."""
        for other in self.balls:
            if (
                ball.id != other.id
                and distance(ball.x, ball.y, other.x, other.y) <= 2 * BALL_RADIUS
            ):
                # Elastic collision response
                dx, dy = other.x - ball.x, other.y - ball.y
                dist = math.sqrt(dx**2 + dy**2)
                nx, ny = dx / dist, dy / dist
                p = (
                    2
                    * (ball.vx * nx + ball.vy * ny - other.vx * nx - other.vy * ny)
                    / 2
                )
                ball.vx -= p * nx
                ball.vy -= p * ny
                other.vx += p * nx
                other.vy += p * ny


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    sim = Simulation(root)
    root.mainloop()
