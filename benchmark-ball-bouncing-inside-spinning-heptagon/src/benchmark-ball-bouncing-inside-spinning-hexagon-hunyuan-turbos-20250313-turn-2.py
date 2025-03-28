import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List
import sys

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
NUM_BALLS = 20
GRAVITY = 0.1
FRICTION = 0.98
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (5 seconds for full rotation)
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

        # Generate heptagon vertices
        self.vertices = self.generate_heptagon_vertices(HEPTAGON_RADIUS)
        self.center = (WIDTH // 2, HEIGHT // 2)

        # Create balls
        self.balls = [
            Ball(
                x=self.center[0], y=self.center[1], vx=0, vy=0,
                spin=0, number=i+1, color=COLORS[i % len(COLORS)]
            )
            for i in range(NUM_BALLS)
        ]

        # Animation loop
        self.frame_rate = 60
        self.last_time = None
        self.animate()

    def generate_heptagon_vertices(self, radius):
        """Generate vertices of a regular heptagon."""
        vertices = []
        for i in range(7):
            angle = 2 * math.pi * i / 7
            x = self.center[0] + radius * math.cos(angle)
            y = self.center[1] + radius * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def get_heptagon_edges(self):
        """Get edges of the heptagon as line segments."""
        edges = []
        for i in range(7):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % 7]
            edges.append((a, b))
        return edges

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

    def check_collision_with_heptagon(self, ball):
        """Check collision between a ball and the heptagon."""
        x, y = ball.x, ball.y
        # Check if the ball is inside the heptagon
        for i in range(7):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % 7]
            if self.is_point_on_segment(x, y, a, b):
                # Ball is on the edge, resolve collision
                normal = self.get_edge_normal(a, b)
                ball.vx -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[0]
                ball.vy -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[1]
                ball.x, ball.y = self.resolve_edge_collision(x, y, a, b)
                return
        # Check if the ball is outside the heptagon
        if self.is_point_outside_heptagon(x, y):
            # Resolve collision by moving the ball inside
            closest_point = self.get_closest_point_on_heptagon(x, y)
            dx, dy = closest_point[0] - x, closest_point[1] - y
            distance = math.hypot(dx, dy)
            if distance > BALL_RADIUS:
                ball.x += dx / distance * BALL_RADIUS
                ball.y += dy / distance * BALL_RADIUS
                # Bounce back
                normal = (dx / distance, dy / distance)
                ball.vx -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[0]
                ball.vy -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[1]

    def is_point_on_segment(self, x, y, a, b):
        """Check if a point is on a line segment."""
        ax, ay = a
        bx, by = b
        cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
        if abs(cross) > BALL_RADIUS:
            return False
        dot = (x - ax) * (bx - ax) + (y - ay) * (by - ay)
        if dot < 0:
            return False
        squared_length = (bx - ax) ** 2 + (by - ay) ** 2
        if dot > squared_length:
            return False
        return True

    def is_point_outside_heptagon(self, x, y):
        """Check if a point is outside the heptagon."""
        for i in range(7):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % 7]
            if self.is_point_on_segment(x, y, a, b):
                return False
        # Ray casting algorithm
        inside = False
        for i in range(7):
            a = self.vertices[i]
            b = self.vertices[(i + 1) % 7]
            if a[1] > b[1]:
                a, b = b, a
            if a[1] <= y < b[1]:
                x_intersect = (y - a[1]) * (b[0] - a[0]) / (b[1] - a[1] + 1e-9) + a[0]
                if x < x_intersect:
                    inside = not inside
        return not inside

    def get_closest_point_on_heptagon(self, x, y):
        """Find the closest point on the heptagon to a given point."""
        closest = self.vertices[0]
        min_distance = math.hypot(x - closest[0], y - closest[1])
        for vertex in self.vertices:
            distance = math.hypot(x - vertex[0], y - vertex[1])
            if distance < min_distance:
                closest = vertex
                min_distance = distance
        return closest

    def get_edge_normal(self, a, b):
        """Get the normal vector of an edge."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        return (-dy, dx)

    def resolve_edge_collision(self, x, y, a, b):
        """Resolve collision by moving the ball to the edge."""
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        if length == 0:
            return x, y
        t = ((x - a[0]) * dx + (y - a[1]) * dy) / (length ** 2)
        t = max(0, min(1, t))
        px = a[0] + t * dx
        py = a[1] + t * dy
        return px, py

    def check_collision_between_balls(self, ball1, ball2):
        """Check collision between two balls."""
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.hypot(dx, dy)
        if distance < BALL_RADIUS * 2:
            # Resolve collision
            angle = math.atan2(dy, dx)
            sine, cosine = math.sin(angle), math.cos(angle)
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
            # Repel balls to avoid overlap
            overlap = BALL_RADIUS * 2 - distance
            push = overlap / 2
            ball1.x -= push * math.cos(angle)
            ball1.y -= push * math.sin(angle)
            ball2.x += push * math.cos(angle)
            ball2.y += push * math.sin(angle)

    def update_ball(self, ball):
        """Update ball position and velocity."""
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
        # Check collision with other balls
        for other in self.balls:
            if ball != other:
                self.check_collision_between_balls(ball, other)

    def animate(self):
        """Main animation loop."""
        if self.last_time is None:
            self.last_time = self.root.after(1000 // self.frame_rate, self.animate)
            return

        current_time = self.root.after_cancel(self.last_time)
        delta_time = (current_time - self.last_time) / 1000
        self.last_time = current_time

        # Rotate heptagon
        self.vertices = [
            self.rotate_point(v, SPIN_SPEED * delta_time, self.center)
            for v in self.vertices
        ]

        # Update and draw balls
        self.canvas.delete("all")
        for ball in self.balls:
            self.update_ball(ball)
            # Draw ball
            self.canvas.create_oval(
                ball.x - BALL_RADIUS, ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS, ball.y + BALL_RADIUS,
                fill=ball.color
            )
            # Draw number
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), fill="black",
                font=("Arial", 10)
            )

        # Schedule next frame
        self.last_time = self.root.after(1000 // self.frame_rate, self.animate)

if __name__ == "__main__":
    root = tk.Tk()
    app = SpinningHeptagonSimulation(root)
    root.mainloop()