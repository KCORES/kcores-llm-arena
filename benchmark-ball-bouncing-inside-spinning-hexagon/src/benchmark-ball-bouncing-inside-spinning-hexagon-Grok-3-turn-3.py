import tkinter as tk
from math import sin, cos, pi, sqrt
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
NUM_BALLS = 20
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.99
HEPTAGON_RADIUS = 300
ROTATION_SPEED = 2 * pi / 5  # 360 degrees in 5 seconds
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
    radius: float
    number: int
    color: str
    angle: float = 0.0
    angular_velocity: float = 0.0


class HeptagonSimulation:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Bouncing Balls in Rotating Heptagon")
        self.canvas = tk.Canvas(self.window, width=800, height=800, bg="white")
        self.canvas.pack()

        self.center_x = 400
        self.center_y = 400
        self.rotation_angle = 0

        # Create balls
        self.balls = [
            Ball(self.center_x, self.center_y, 0, 0, BALL_RADIUS, i + 1, COLORS[i])
            for i in range(NUM_BALLS)
        ]

        # Calculate heptagon vertices
        self.vertices = self.calculate_heptagon_vertices()

        self.update()
        self.window.mainloop()

    def calculate_heptagon_vertices(self) -> List[tuple]:
        vertices = []
        for i in range(7):
            angle = 2 * pi * i / 7
            x = self.center_x + HEPTAGON_RADIUS * cos(angle)
            y = self.center_y + HEPTAGON_RADIUS * sin(angle)
            vertices.append((x, y))
        return vertices

    def rotate_heptagon(self):
        self.rotation_angle += ROTATION_SPEED / 60  # Assuming 60 FPS
        rotated_vertices = []
        for x, y in self.vertices:
            # Translate to origin
            tx = x - self.center_x
            ty = y - self.center_y
            # Rotate
            rx = tx * cos(self.rotation_angle) - ty * sin(self.rotation_angle)
            ry = tx * sin(self.rotation_angle) + ty * cos(self.rotation_angle)
            # Translate back
            rotated_vertices.append((rx + self.center_x, ry + self.center_y))
        return rotated_vertices

    def point_line_distance(self, px, py, x1, y1, x2, y2):
        # Vector from line start to point
        line_vec = np.array([x2 - x1, y2 - y1])
        point_vec = np.array([px - x1, py - y1])

        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return np.linalg.norm(point_vec)

        line_unit = line_vec / line_len
        t = max(0, min(1, np.dot(point_vec, line_unit)))
        projection = np.array([x1, y1]) + t * line_vec
        return np.linalg.norm(np.array([px, py]) - projection)

    def ball_wall_collision(self, ball: Ball, vertices: List[tuple]):
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]

            dist = self.point_line_distance(ball.x, ball.y, v1[0], v1[1], v2[0], v2[1])
            if dist <= ball.radius:
                # Calculate wall normal
                wall_vec = np.array([v2[0] - v1[0], v2[1] - v1[1]])
                normal = np.array([-wall_vec[1], wall_vec[0]])
                normal = normal / np.linalg.norm(normal)

                # Velocity vector
                vel = np.array([ball.vx, ball.vy])

                # Reflect velocity
                dot_product = np.dot(vel, normal)
                if dot_product < 0:  # Only reflect if moving towards wall
                    reflected = vel - 2 * dot_product * normal
                    ball.vx, ball.vy = reflected * 0.8  # Some energy loss

                    # Push ball out of wall
                    penetration = ball.radius - dist
                    ball.x += normal[0] * penetration
                    ball.y += normal[1] * penetration

                    # Add rotation from friction
                    ball.angular_velocity += dot_product * 0.1 / ball.radius

    def ball_ball_collision(self, ball1: Ball, ball2: Ball):
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

            # Velocity component in collision direction
            v_rel = dvx * nx + dvy * ny

            if v_rel < 0:  # Balls moving toward each other
                # Impulse calculation (perfectly elastic collision)
                impulse = 2 * v_rel / 2  # Equal masses

                # Update velocities
                ball1.vx += impulse * nx
                ball1.vy += impulse * ny
                ball2.vx -= impulse * nx
                ball2.vy -= impulse * ny

                # Separate overlapping balls
                overlap = 2 * ball1.radius - distance
                ball1.x -= nx * overlap / 2
                ball1.y -= ny * overlap / 2
                ball2.x += nx * overlap / 2
                ball2.y += ny * overlap / 2

                # Add some rotational effect
                ball1.angular_velocity += v_rel * 0.05 / ball1.radius
                ball2.angular_velocity -= v_rel * 0.05 / ball2.radius

    def update(self):
        self.canvas.delete("all")

        # Update physics
        rotated_vertices = self.rotate_heptagon()

        # Draw heptagon
        self.canvas.create_polygon(rotated_vertices, outline="black", width=2)

        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY

            # Apply friction to velocity
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.angular_velocity *= FRICTION

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy
            ball.angle += ball.angular_velocity

            # Check wall collisions
            self.ball_wall_collision(ball, rotated_vertices)

        # Check ball-ball collisions
        for i in range(NUM_BALLS):
            for j in range(i + 1, NUM_BALLS):
                self.ball_ball_collision(self.balls[i], self.balls[j])

        # Draw balls
        for ball in self.balls:
            # Draw ball
            self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
            )

            # Draw number (rotated)
            x1 = ball.x + ball.radius * cos(ball.angle)
            y1 = ball.y + ball.radius * sin(ball.angle)
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                font=("Arial", 12, "bold"),
                fill="white",
            )

        self.window.after(16, self.update)  # ~60 FPS


if __name__ == "__main__":
    HeptagonSimulation()
