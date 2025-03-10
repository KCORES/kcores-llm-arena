import tkinter as tk
from math import sin, cos, radians, sqrt
import numpy as np
from dataclasses import dataclass
from typing import List
import sys

# Constants
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
BALL_RADIUS = 15
NUM_BALLS = 20
GRAVITY = 0.5
FRICTION = 0.98
HEPTAGON_RADIUS = 300
ROTATION_SPEED = 360 / 5  # degrees per second


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0
    angular_vel: float = 0


class HeptagonSimulation:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Bouncing Balls in Rotating Heptagon")
        self.canvas = tk.Canvas(self.window, width=800, height=800, bg="white")
        self.canvas.pack()

        self.center_x = 400
        self.center_y = 400
        self.heptagon_angle = 0

        # Initialize balls
        self.balls = []
        for i in range(NUM_BALLS):
            ball = Ball(
                x=self.center_x,
                y=self.center_y,
                vx=0,
                vy=0,
                radius=BALL_RADIUS,
                color=COLORS[i],
                number=i + 1,
            )
            self.balls.append(ball)

        self.last_time = 0
        self.update()

    def get_heptagon_vertices(self):
        vertices = []
        for i in range(7):
            angle = radians(self.heptagon_angle + i * 360 / 7)
            x = self.center_x + HEPTAGON_RADIUS * cos(angle)
            y = self.center_y + HEPTAGON_RADIUS * sin(angle)
            vertices.append((x, y))
        return vertices

    def point_line_distance(self, px, py, x1, y1, x2, y2):
        line_vec = np.array([x2 - x1, y2 - y1])
        point_vec = np.array([px - x1, py - y1])
        line_len = np.linalg.norm(line_vec)
        line_unit = line_vec / line_len if line_len > 0 else line_vec
        projection = np.dot(point_vec, line_unit)
        if projection < 0:
            return sqrt((px - x1) ** 2 + (py - y1) ** 2)
        elif projection > line_len:
            return sqrt((px - x2) ** 2 + (py - y2) ** 2)
        closest = np.array([x1, y1]) + projection * line_unit
        return sqrt((px - closest[0]) ** 2 + (py - closest[1]) ** 2)

    def ball_wall_collision(self, ball: Ball, vertices: List[tuple]):
        for i in range(7):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 7]

            dist = self.point_line_distance(ball.x, ball.y, x1, y1, x2, y2)
            if dist <= ball.radius:
                # Normal vector
                wall_vec = np.array([x2 - x1, y2 - y1])
                normal = np.array([-wall_vec[1], wall_vec[0]])
                normal = normal / np.linalg.norm(normal)

                # Velocity vector
                vel = np.array([ball.vx, ball.vy])
                vel_normal = np.dot(vel, normal)

                if vel_normal < 0:  # Moving towards wall
                    # Reflect velocity
                    ball.vx -= 2 * vel_normal * normal[0] * 0.8
                    ball.vy -= 2 * vel_normal * normal[1] * 0.8

                    # Push ball out
                    penetration = ball.radius - dist
                    ball.x += normal[0] * penetration
                    ball.y += normal[1] * penetration

                    # Add rotation from friction
                    ball.angular_vel += vel_normal * 0.1 / ball.radius

    def ball_ball_collision(self, ball1: Ball, ball2: Ball):
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        dist = sqrt(dx * dx + dy * dy)

        if dist < ball1.radius + ball2.radius and dist > 0:
            # Normal vector
            normal = np.array([dx / dist, dy / dist])
            relative_v = np.array([ball2.vx - ball1.vx, ball2.vy - ball1.vy])
            vel_normal = np.dot(relative_v, normal)

            if vel_normal < 0:  # Moving towards each other
                # Impulse calculation (elastic collision)
                impulse = 2 * vel_normal / 2  # equal mass
                ball1.vx += impulse * normal[0] * 0.8
                ball1.vy += impulse * normal[1] * 0.8
                ball2.vx -= impulse * normal[0] * 0.8
                ball2.vy -= impulse * normal[1] * 0.8

                # Separate balls
                overlap = (ball1.radius + ball2.radius - dist) / 2
                ball1.x -= normal[0] * overlap
                ball1.y -= normal[1] * overlap
                ball2.x += normal[0] * overlap
                ball2.y += normal[1] * overlap

    def update(self):
        self.canvas.delete("all")

        # Update time
        current_time = self.window.tk.call("clock", "milliseconds") / 1000
        dt = current_time - self.last_time if self.last_time else 0.016
        self.last_time = current_time

        # Update heptagon rotation
        self.heptagon_angle += ROTATION_SPEED * dt
        vertices = self.get_heptagon_vertices()

        # Draw heptagon
        self.canvas.create_polygon(vertices, outline="black", fill="lightgray")

        # Update balls
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY * dt

            # Apply friction to velocity
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt

            # Update rotation
            ball.angle += ball.angular_vel * dt
            ball.angular_vel *= FRICTION

            # Check wall collisions
            self.ball_wall_collision(ball, vertices)

        # Check ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
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
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                font=("Arial", 12, "bold"),
                angle=ball.angle * 180 / 3.14159,
            )

        self.window.after(16, self.update)  # ~60 FPS

    def run(self):
        self.window.mainloop()


if __name__ == "__main__":
    sim = HeptagonSimulation()
    sim.run()
