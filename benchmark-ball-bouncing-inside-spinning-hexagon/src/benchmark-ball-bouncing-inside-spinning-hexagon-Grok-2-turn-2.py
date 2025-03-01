import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
NUM_BALLS = 20
BALL_RADIUS = 10
GRAVITY = 0.5
FRICTION = 0.98
HEPTAGON_RADIUS = 300  # Distance from center to vertex
ROTATION_SPEED = 360 / 5  # Degrees per second
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
FPS = 60


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0.0  # Rotation angle for number
    angular_velocity: float = 0.0


class HeptagonSimulation:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(self.window, width=800, height=800, bg="white")
        self.canvas.pack()

        # Initialize balls
        self.balls = [
            Ball(400, 400, 0, 0, BALL_RADIUS, COLORS[i], i + 1)
            for i in range(NUM_BALLS)
        ]

        self.heptagon_angle = 0
        self.time_step = 1 / FPS
        self.running = True
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.update()

    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        """Calculate current heptagon vertices based on rotation"""
        center_x, center_y = 400, 400
        vertices = []
        for i in range(7):
            angle = math.radians(self.heptagon_angle + (360 / 7) * i)
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def point_line_distance(
        self, p: Tuple[float, float], v1: Tuple[float, float], v2: Tuple[float, float]
    ) -> Tuple[float, Tuple[float, float]]:
        """Calculate distance from point to line segment and closest point"""
        x, y = p
        x1, y1 = v1
        x2, y2 = v2

        # Line segment vector
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy

        if length_sq == 0:
            dist = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
            return dist, (x1, y1)

        # Project point onto line
        t = max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / length_sq))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        dist = math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)
        return dist, (closest_x, closest_y)

    def ball_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        """Handle collision with heptagon walls"""
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]
            dist, closest = self.point_line_distance((ball.x, ball.y), v1, v2)

            if dist <= ball.radius:
                # Calculate normal vector
                wall_vec = (v2[0] - v1[0], v2[1] - v1[1])
                wall_len = math.sqrt(wall_vec[0] ** 2 + wall_vec[1] ** 2)
                normal = (-wall_vec[1] / wall_len, wall_vec[0] / wall_len)

                # Calculate velocity relative to wall
                vel_dot_normal = ball.vx * normal[0] + ball.vy * normal[1]

                if vel_dot_normal < 0:  # Moving towards wall
                    # Reflect velocity
                    ball.vx -= 2 * vel_dot_normal * normal[0] * 0.8
                    ball.vy -= 2 * vel_dot_normal * normal[1] * 0.8

                    # Add angular velocity from friction
                    ball.angular_velocity += vel_dot_normal * 0.1

                    # Push ball out of wall
                    penetration = ball.radius - dist
                    ball.x += normal[0] * penetration
                    ball.y += normal[1] * penetration

    def ball_ball_collision(self, ball1: Ball, ball2: Ball):
        """Handle collision between two balls"""
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < 2 * ball1.radius and distance > 0:
            # Normalize collision vector
            nx = dx / distance
            ny = dy / distance

            # Relative velocity
            rvx = ball2.vx - ball1.vx
            rvy = ball2.vy - ball1.vy
            vel_normal = rvx * nx + rvy * ny

            if vel_normal < 0:  # Moving towards each other
                # Impulse calculation (elastic collision)
                impulse = 2 * vel_normal / 2  # Equal mass
                ball1.vx += impulse * nx * 0.8
                ball1.vy += impulse * ny * 0.8
                ball2.vx -= impulse * nx * 0.8
                ball2.vy -= impulse * ny * 0.8

                # Add angular velocity
                ball1.angular_velocity -= vel_normal * 0.05
                ball2.angular_velocity += vel_normal * 0.05

                # Separate overlapping balls
                overlap = 2 * ball1.radius - distance
                ball1.x -= nx * overlap / 2
                ball1.y -= ny * overlap / 2
                ball2.x += nx * overlap / 2
                ball2.y += ny * overlap / 2

    def update(self):
        if not self.running:
            return

        # Update heptagon rotation
        self.heptagon_angle += ROTATION_SPEED * self.time_step
        vertices = self.get_heptagon_vertices()

        # Update balls
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY

            # Apply friction to velocity
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Update rotation
            ball.angle += ball.angular_velocity
            ball.angular_velocity *= FRICTION

        # Check collisions
        for i, ball in enumerate(self.balls):
            self.ball_wall_collision(ball, vertices)
            for j in range(i + 1, NUM_BALLS):
                self.ball_ball_collision(ball, self.balls[j])

        # Draw
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(vertices, outline="black", fill="")

        # Draw balls
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius

            # Draw ball
            oval = self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)

            # Draw number (rotated)
            cx, cy = ball.x, ball.y
            num_x = cx + math.cos(ball.angle) * ball.radius * 0.5
            num_y = cy + math.sin(ball.angle) * ball.radius * 0.5
            self.canvas.create_text(
                num_x,
                num_y,
                text=str(ball.number),
                fill="black",
                font=("Arial", 10, "bold"),
            )

        self.window.after(int(1000 / FPS), self.update)

    def close(self):
        self.running = False
        self.window.destroy()


if __name__ == "__main__":
    sim = HeptagonSimulation()
    sim.window.mainloop()
