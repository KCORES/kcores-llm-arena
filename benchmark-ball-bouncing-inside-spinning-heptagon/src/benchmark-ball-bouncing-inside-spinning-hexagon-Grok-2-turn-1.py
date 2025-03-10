import tkinter as tk
from math import sin, cos, pi, sqrt
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    number: int
    angle: float
    angular_vel: float


class HeptagonSimulation:
    def __init__(self):
        self.width = 800
        self.height = 800
        self.root = tk.Tk()
        self.canvas = tk.Canvas(
            self.root, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        # Colors for balls
        self.colors = [
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

        # Heptagon parameters
        self.heptagon_center = (self.width / 2, self.height / 2)
        self.heptagon_radius = 300
        self.heptagon_angle = 0
        self.angular_speed = 2 * pi / 5  # 360 degrees in 5 seconds

        # Ball parameters
        self.ball_radius = 15
        self.balls: List[Ball] = []
        self.gravity = 0.5
        self.friction = 0.99
        self.elasticity = 0.7

        # Initialize balls
        for i in range(20):
            ball = Ball(
                x=self.width / 2,
                y=self.height / 2,
                vx=0,
                vy=0,
                radius=self.ball_radius,
                number=i + 1,
                angle=0,
                angular_vel=0,
            )
            self.balls.append(ball)

        self.setup_simulation()

    def heptagon_vertices(self) -> List[Tuple[float, float]]:
        """Calculate current heptagon vertices based on rotation"""
        vertices = []
        for i in range(7):
            angle = self.heptagon_angle + (2 * pi * i / 7)
            x = self.heptagon_center[0] + self.heptagon_radius * cos(angle)
            y = self.heptagon_center[1] + self.heptagon_radius * sin(angle)
            vertices.append((x, y))
        return vertices

    def line_intersection(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        p4: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Find intersection point of two lines"""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None

        px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        return (px, py)

    def ball_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        """Handle collision with heptagon walls"""
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]

            # Vector from ball center to vertex
            dx = ball.x - v1[0]
            dy = ball.y - v1[1]

            # Wall vector
            wall_dx = v2[0] - v1[0]
            wall_dy = v2[1] - v1[1]

            # Distance from ball to wall
            dist = abs(dx * wall_dy - dy * wall_dx) / sqrt(wall_dx**2 + wall_dy**2)

            if dist <= ball.radius:
                # Normal vector
                normal = np.array([-wall_dy, wall_dx])
                normal = normal / np.linalg.norm(normal)

                # Velocity vector
                vel = np.array([ball.vx, ball.vy])

                # Reflect velocity
                dot_product = np.dot(vel, normal)
                if dot_product < 0:  # Only reflect if moving towards wall
                    reflected = vel - 2 * dot_product * normal
                    ball.vx = reflected[0] * self.elasticity
                    ball.vy = reflected[1] * self.elasticity

                    # Add angular velocity based on impact
                    ball.angular_vel += dot_product / ball.radius

    def ball_ball_collision(self, ball1: Ball, ball2: Ball):
        """Handle collision between balls"""
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = sqrt(dx * dx + dy * dy)

        if distance < 2 * ball1.radius and distance > 0:
            # Normalize collision vector
            nx = dx / distance
            ny = dy / distance

            # Relative velocity
            dvx = ball2.vx - ball1.vx
            dvy = ball2.vy - ball1.vy

            # Impulse magnitude
            impulse = 2 * (dvx * nx + dvy * ny) / 2  # 2 balls of equal mass

            if impulse < 0:  # Only if moving towards each other
                # Apply impulse
                ball1.vx += impulse * nx * self.elasticity
                ball1.vy += impulse * ny * self.elasticity
                ball2.vx -= impulse * nx * self.elasticity
                ball2.vy -= impulse * ny * self.elasticity

                # Add rotational impulse
                ball1.angular_vel += impulse / ball1.radius
                ball2.angular_vel -= impulse / ball2.radius

                # Separate overlapping balls
                overlap = 2 * ball1.radius - distance
                ball1.x -= nx * overlap / 2
                ball1.y -= ny * overlap / 2
                ball2.x += nx * overlap / 2
                ball2.y += ny * overlap / 2

    def update(self):
        # Update heptagon rotation
        self.heptagon_angle += self.angular_speed * 0.016  # 60 FPS assumption

        # Get current vertices
        vertices = self.heptagon_vertices()

        # Update balls
        for ball in self.balls:
            # Apply gravity
            ball.vy += self.gravity

            # Apply friction
            ball.vx *= self.friction
            ball.vy *= self.friction
            ball.angular_vel *= self.friction

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy
            ball.angle += ball.angular_vel

            # Check wall collision
            self.ball_wall_collision(ball, vertices)

        # Check ball-ball collisions
        for i, ball1 in enumerate(self.balls):
            for ball2 in self.balls[i + 1 :]:
                self.ball_ball_collision(ball1, ball2)

        # Redraw
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(vertices, outline="black", width=2)

        # Draw balls
        for i, ball in enumerate(self.balls):
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius

            # Draw ball
            oval = self.canvas.create_oval(x1, y1, x2, y2, fill=self.colors[i])

            # Draw number (rotated)
            cx, cy = ball.x, ball.y
            num_x = cx + cos(ball.angle) * ball.radius * 0.5
            num_y = cy + sin(ball.angle) * ball.radius * 0.5
            self.canvas.create_text(
                num_x, num_y, text=str(ball.number), font=("Arial", 12)
            )

        self.root.after(16, self.update)  # ~60 FPS

    def setup_simulation(self):
        self.root.title("Bouncing Balls in Spinning Heptagon")
        self.update()
        self.root.mainloop()


if __name__ == "__main__":
    sim = HeptagonSimulation()
