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
NUM_BALLS = 20
GRAVITY = 0.1
FRICTION = 0.98
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (5 seconds for full rotation)

# Colors for the balls
BALL_COLORS = [
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

        # Heptagon vertices
        self.heptagon_vertices = self.calculate_heptagon_vertices(HEPTAGON_RADIUS)
        self.center = (WIDTH // 2, HEIGHT // 2)

        # Initialize balls
        self.balls: List[Ball] = [
            Ball(
                x=self.center[0], y=self.center[1], vx=0, vy=0,
                spin=0, number=i+1, color=BALL_COLORS[i % len(BALL_COLORS)]
            )
            for i in range(NUM_BALLS)
        ]

        # Animation loop
        self.animate()

    def calculate_heptagon_vertices(self, radius):
        """Calculate the vertices of a regular heptagon."""
        vertices = []
        for i in range(7):
            angle = 2 * math.pi * i / 7
            x = self.center[0] + radius * math.cos(angle)
            y = self.center[1] + radius * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def rotate_point(self, point, angle, center):
        """Rotate a point around a center by a given angle."""
        x, y = point
        cx, cy = center
        angle_rad = math.radians(angle)
        cos_theta = math.cos(angle_rad)
        sin_theta = math.sin(angle_rad)
        x_new = cx + (x - cx) * cos_theta - (y - cy) * sin_theta
        y_new = cy + (x - cx) * sin_theta + (y - cy) * cos_theta
        return x_new, y_new

    def is_inside_heptagon(self, x, y):
        """Check if a point is inside the heptagon."""
        for i in range(len(self.heptagon_vertices)):
            a = self.heptagon_vertices[i]
            b = self.heptagon_vertices[(i + 1) % len(self.heptagon_vertices)]
            if self.is_point_on_segment(a, b, (x, y)):
                return False
        return True

    def is_point_on_segment(self, a, b, c):
        """Check if point c lies on segment ab."""
        cross_product = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])
        if abs(cross_product) > 1e-12:
            return False
        dot_product = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1]) * (b[1] - a[1])
        if dot_product < 0:
            return False
        squared_length = (b[0] - a[0])**2 + (b[1] - a[1])**2
        if dot_product > squared_length:
            return False
        return True

    def check_collision_with_heptagon(self, ball):
        """Check if a ball is colliding with the heptagon and resolve it."""
        for i in range(len(self.heptagon_vertices)):
            a = self.rotate_point(self.heptagon_vertices[i], SPIN_SPEED, self.center)
            b = self.rotate_point(self.heptagon_vertices[(i + 1) % len(self.heptagon_vertices)], SPIN_SPEED, self.center)
            if self.line_circle_collision(a, b, ball.x, ball.y, BALL_RADIUS):
                # Resolve collision
                normal = (b[0] - a[0], b[1] - a[1])
                length = math.hypot(normal[0], normal[1])
                normal = (normal[0] / length, normal[1] / length)
                relative_velocity = (ball.vx, ball.vy)
                velocity_along_normal = relative_velocity[0] * normal[0] + relative_velocity[1] * normal[1]
                if velocity_along_normal > 0:
                    continue
                e = 1.0  # Coefficient of restitution
                j = -(1 + e) * velocity_along_normal
                j /= (1 / BALL_RADIUS + 1 / BALL_RADIUS)
                impulse = (j * normal[0], j * normal[1])
                ball.vx -= impulse[0]
                ball.vy -= impulse[1]

    def line_circle_collision(self, a, b, cx, cy, radius):
        """Check if a line segment ab intersects a circle at (cx, cy) with given radius."""
        ap = (cx - a[0], cy - a[1])
        ab = (b[0] - a[0], b[1] - a[1])
        t = (ap[0] * ab[0] + ap[1] * ab[1]) / (ab[0]**2 + ab[1]**2 + 1e-12)
        t = max(0, min(1, t))
        closest = (a[0] + t * ab[0], a[1] + t * ab[1])
        dx = cx - closest[0]
        dy = cy - closest[1]
        return dx**2 + dy**2 <= radius**2

    def check_ball_collisions(self):
        """Check for collisions between balls and resolve them."""
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.hypot(dx, dy)
                if distance < 2 * BALL_RADIUS:
                    # Resolve collision
                    angle = math.atan2(dy, dx)
                    sine = math.sin(angle)
                    cosine = math.cos(angle)
                    # Rotate ball1's velocity
                    vx1 = ball1.vx * cosine + ball1.vy * sine
                    vy1 = ball1.vy * cosine - ball1.vx * sine
                    vx2 = ball2.vx * cosine + ball2.vy * sine
                    vy2 = ball2.vy * cosine - ball2.vx * sine
                    # Exchange velocities along x-axis
                    vx1_new = (vx1 * (BALL_RADIUS - BALL_RADIUS) + 2 * BALL_RADIUS * vx2) / (BALL_RADIUS + BALL_RADIUS)
                    vx2_new = (vx2 * (BALL_RADIUS - BALL_RADIUS) + 2 * BALL_RADIUS * vx1) / (BALL_RADIUS + BALL_RADIUS)
                    # Rotate back velocities
                    ball1.vx = vx1_new * cosine - vy1 * sine
                    ball1.vy = vy1 * cosine + vx1_new * sine
                    ball2.vx = vx2_new * cosine - vy2 * sine
                    ball2.vy = vy2 * cosine + vx2_new * sine
                    # Move balls apart
                    overlap = 2 * BALL_RADIUS - distance
                    correction = overlap / 2
                    ball1.x -= correction * math.cos(angle)
                    ball1.y -= correction * math.sin(angle)
                    ball2.x += correction * math.cos(angle)
                    ball2.y += correction * math.sin(angle)

    def update(self):
        """Update the positions and velocities of the balls."""
        self.canvas.delete("all")
        self.draw_heptagon()

        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY

            # Apply friction
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Check collision with heptagon
            self.check_collision_with_heptagon(ball)

            # Check collision with walls
            if ball.x - BALL_RADIUS < 0 or ball.x + BALL_RADIUS > WIDTH:
                ball.vx *= -1
                ball.x = BALL_RADIUS if ball.x < BALL_RADIUS else WIDTH - BALL_RADIUS
            if ball.y - BALL_RADIUS < 0 or ball.y + BALL_RADIUS > HEIGHT:
                ball.vy *= -1
                ball.y = BALL_RADIUS if ball.y < BALL_RADIUS else HEIGHT - BALL_RADIUS

            # Check collision with other balls
            self.check_ball_collisions()

            # Draw ball
            self.canvas.create_oval(
                ball.x - BALL_RADIUS, ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS, ball.y + BALL_RADIUS,
                fill=ball.color
            )
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), fill="black"
            )

    def draw_heptagon(self):
        """Draw the spinning heptagon."""
        for i in range(len(self.heptagon_vertices)):
            a = self.rotate_point(self.heptagon_vertices[i], SPIN_SPEED, self.center)
            b = self.rotate_point(self.heptagon_vertices[(i + 1) % len(self.heptagon_vertices)], SPIN_SPEED, self.center)
            self.canvas.create_line(a[0], a[1], b[0], b[1], fill="white")

    def animate(self):
        """Animate the simulation."""
        self.update()
        self.root.after(16, self.animate)  # ~60 FPS

if __name__ == "__main__":
    root = tk.Tk()
    app = SpinningHeptagonSimulation(root)
    root.mainloop()