import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List
import sys

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 15
GRAVITY = 0.1
FRICTION = 0.98
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (5 seconds for full rotation)
NUM_BALLS = 20
COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51",
    "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800",
    "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61",
    "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
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

class SpinningHeptagonSimulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.balls: List[Ball] = []
        self.heptagon_angle = 0  # Current rotation angle of the heptagon
        self.init_balls()
        self.init_heptagon()

        self.update()

    def init_balls(self):
        for i in range(NUM_BALLS):
            angle = 2 * math.pi * i / NUM_BALLS
            r = HEPTAGON_RADIUS - BALL_RADIUS
            x = WIDTH / 2 + r * math.cos(angle)
            y = HEIGHT / 2 + r * math.sin(angle)
            vx, vy = 0, 0
            spin = 0
            color = COLORS[i % len(COLORS)]
            ball = Ball(x, y, vx, vy, spin, i + 1, color)
            self.balls.append(ball)

    def init_heptagon(self):
        self.heptagon_points = []
        for i in range(7):
            angle = 2 * math.pi * i / 7
            x = WIDTH / 2 + HEPTAGON_RADIUS * math.cos(angle)
            y = HEIGHT / 2 + HEPTAGON_RADIUS * math.sin(angle)
            self.heptagon_points.append((x, y))

    def rotate_point(self, x, y, angle):
        """Rotate a point (x, y) by a given angle around the center (WIDTH/2, HEIGHT/2)."""
        cx, cy = WIDTH / 2, HEIGHT / 2
        cos_theta = math.cos(math.radians(angle))
        sin_theta = math.sin(math.radians(angle))
        x_new = cx + (x - cx) * cos_theta - (y - cy) * sin_theta
        y_new = cy + (x - cx) * sin_theta + (y - cy) * cos_theta
        return x_new, y_new

    def get_heptagon_edges(self):
        """Get the edges of the heptagon, rotated by the current angle."""
        edges = []
        for i in range(7):
            a = self.rotate_point(*self.heptagon_points[i], self.heptagon_angle)
            b = self.rotate_point(*self.heptagon_points[(i + 1) % 7], self.heptagon_angle)
            edges.append((a, b))
        return edges

    def check_collision_with_heptagon(self, ball):
        """Check for collisions between a ball and the heptagon edges."""
        edges = self.get_heptagon_edges()
        for a, b in edges:
            if self.line_circle_collision(a, b, ball.x, ball.y, BALL_RADIUS):
                # Handle collision response
                normal = self.get_normal(a, b, ball.x, ball.y)
                ball.vx -= 2 * ball.vx * normal[0] + 2 * ball.vy * normal[1]
                ball.vy -= 2 * ball.vx * normal[1] - 2 * ball.vy * normal[0]
                ball.x, ball.y = self.resolve_collision(ball, a, b)

    def line_circle_collision(self, a, b, cx, cy, r):
        """Check if a circle (cx, cy, r) collides with a line segment (a, b)."""
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        fx, fy = cx - ax, cy - ay

        t = (fx * dx + fy * dy) / (dx**2 + dy**2 + 1e-6)
        t = max(0, min(1, t))
        closest_x, closest_y = ax + t * dx, ay + t * dy

        dist_sq = (closest_x - cx)**2 + (closest_y - cy)**2
        return dist_sq <= r**2

    def get_normal(self, a, b, cx, cy):
        """Get the normal vector at the collision point."""
        ax, ay = a
        bx, by = b
        nx, ny = by - ay, ax - bx  # Perpendicular vector
        length = math.hypot(nx, ny)
        if length == 0:
            return (0, 0)
        nx /= length
        ny /= length
        dot = (cx - ax) * nx + (cy - ay) * ny
        return (nx * dot, ny * dot)

    def resolve_collision(self, ball, a, b):
        """Resolve the collision by moving the ball slightly outside the heptagon."""
        closest_a = self.closest_point_on_segment(a, b, ball.x, ball.y)
        dx, dy = ball.x - closest_a[0], ball.y - closest_a[1]
        distance = math.hypot(dx, dy)
        overlap = BALL_RADIUS - distance
        if overlap > 0:
            scale = overlap / distance
            ball.x = closest_a[0] + dx * scale
            ball.y = closest_a[1] + dy * scale
        return ball.x, ball.y

    def closest_point_on_segment(self, a, b, cx, cy):
        """Find the closest point on a line segment (a, b) to a point (cx, cy)."""
        ax, ay = a
        bx, by = b
        ap_x, ap_y = cx - ax, cy - ay
        ab_x, ab_y = bx - ax, by - ay
        t = (ap_x * ab_x + ap_y * ab_y) / (ab_x**2 + ab_y**2 + 1e-6)
        t = max(0, min(1, t))
        return ax + t * ab_x, ay + t * ab_y

    def check_ball_collisions(self):
        """Check for collisions between balls."""
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                dx, dy = ball2.x - ball1.x, ball2.y - ball1.y
                dist_sq = dx**2 + dy**2
                sum_radius = BALL_RADIUS * 2
                if dist_sq < sum_radius**2:
                    # Handle collision response
                    angle = math.atan2(dy, dx)
                    total_velocity_x = ball1.vx + ball2.vx
                    total_velocity_y = ball1.vy + ball2.vy
                    ball1.vx, ball2.vx = self.resolve_ball_collision(ball1, ball2, angle, total_velocity_x, total_velocity_y)
                    ball1.vy, ball2.vy = self.resolve_ball_collision(ball1, ball2, angle, total_velocity_y, total_velocity_x)

    def resolve_ball_collision(self, ball1, ball2, angle, vx_total, vy_total):
        """Resolve collision between two balls."""
        mass1, mass2 = 1, 1  # Assume equal mass for simplicity
        ball1.vx = (vx_total * (mass1 - mass2) + 2 * mass2 * ball2.vx) / (mass1 + mass2)
        ball2.vx = (vx_total * (mass2 - mass1) + 2 * mass1 * ball1.vx) / (mass1 + mass2)
        ball1.vy = (vy_total * (mass1 - mass2) + 2 * mass2 * ball2.vy) / (mass1 + mass2)
        ball2.vy = (vy_total * (mass2 - mass1) + 2 * mass1 * ball1.vy) / (mass1 + mass2)
        return ball1.vx, ball2.vx

    def update(self):
        """Update the simulation."""
        self.canvas.delete("all")

        # Update heptagon rotation
        self.heptagon_angle += SPIN_SPEED
        if self.heptagon_angle >= 360:
            self.heptagon_angle -= 360

        # Draw heptagon
        self.draw_heptagon()

        # Update and draw balls
        for ball in self.balls:
            ball.vy += GRAVITY * FRICTION
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.x += ball.vx
            ball.y += ball.vy

            # Collision with walls
            self.check_collision_with_heptagon(ball)

            # Collision between balls
            self.check_ball_collisions()

            # Draw ball
            self.draw_ball(ball)

        # Repeat the update
        self.root.after(16, self.update)  # ~60 FPS

    def draw_heptagon(self):
        """Draw the spinning heptagon."""
        edges = self.get_heptagon_edges()
        for a, b in edges:
            x1, y1 = int(a[0]), int(a[1])
            x2, y2 = int(b[0]), int(b[1])
            self.canvas.create_line(x1, y1, x2, y2, fill="white", width=2)

    def draw_ball(self, ball):
        """Draw a ball on the canvas."""
        x, y = int(ball.x), int(ball.y)
        self.canvas.create_oval(x - BALL_RADIUS, y - BALL_RADIUS, x + BALL_RADIUS, y + BALL_RADIUS, fill=ball.color)
        self.canvas.create_text(x, y, text=str(ball.number), fill="black", font=("Arial", 8))

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Spinning Heptagon with Balls")
    app = SpinningHeptagonSimulation(root)
    root.mainloop()