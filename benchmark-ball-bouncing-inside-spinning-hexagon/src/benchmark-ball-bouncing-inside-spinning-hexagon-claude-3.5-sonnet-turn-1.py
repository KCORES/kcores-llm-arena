import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys


@dataclass
class Ball:
    x: float  # center x position
    y: float  # center y position
    vx: float  # velocity x
    vy: float  # velocity y
    angle: float  # rotation angle
    angular_vel: float  # angular velocity
    color: str
    number: int


class BouncingBallsApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in Spinning Heptagon")

        # Constants
        self.WIDTH = 800
        self.HEIGHT = 800
        self.CENTER = (self.WIDTH / 2, self.HEIGHT / 2)
        self.HEPTAGON_RADIUS = 300
        self.BALL_RADIUS = 20
        self.GRAVITY = 981.0  # pixels/s^2
        self.FRICTION = 0.98
        self.ANGULAR_FRICTION = 0.99
        self.RESTITUTION = 0.7
        self.HEPTAGON_ANGULAR_SPEED = 2 * math.pi / 5  # radians per second

        self.canvas = tk.Canvas(self.root, width=self.WIDTH, height=self.HEIGHT)
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

        self.heptagon_angle = 0
        self.balls = self.initialize_balls()
        self.last_update = None

        self.update()
        self.root.mainloop()

    def initialize_balls(self) -> List[Ball]:
        balls = []
        for i in range(20):
            angle = 2 * math.pi * i / 20
            # All balls start from center with slight random velocities
            balls.append(
                Ball(
                    x=self.CENTER[0],
                    y=self.CENTER[1],
                    vx=np.random.uniform(-50, 50),
                    vy=np.random.uniform(-50, 50),
                    angle=0,
                    angular_vel=0,
                    color=self.colors[i],
                    number=i + 1,
                )
            )
        return balls

    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        for i in range(7):
            angle = self.heptagon_angle + 2 * math.pi * i / 7
            x = self.CENTER[0] + self.HEPTAGON_RADIUS * math.cos(angle)
            y = self.CENTER[1] + self.HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def check_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]

            # Vector from wall start to ball center
            wall_to_ball = np.array([ball.x - v1[0], ball.y - v1[1]])
            # Wall vector
            wall = np.array([v2[0] - v1[0], v2[1] - v1[1]])
            wall_len = np.linalg.norm(wall)
            wall_unit = wall / wall_len

            # Project ball position onto wall
            proj_length = np.dot(wall_to_ball, wall_unit)

            if 0 <= proj_length <= wall_len:
                # Find closest point on wall
                closest = np.array([v1[0], v1[1]]) + wall_unit * proj_length
                dist = np.linalg.norm(np.array([ball.x, ball.y]) - closest)

                if dist < self.BALL_RADIUS:
                    # Calculate normal vector
                    normal = np.array([wall_unit[1], -wall_unit[0]])
                    if np.dot(normal, wall_to_ball) < 0:
                        normal = -normal

                    # Reflect velocity
                    velocity = np.array([ball.vx, ball.vy])
                    reflected = velocity - 2 * np.dot(velocity, normal) * normal

                    # Apply restitution and friction
                    ball.vx = reflected[0] * self.RESTITUTION
                    ball.vy = reflected[1] * self.RESTITUTION

                    # Update angular velocity based on impact
                    impact_speed = np.linalg.norm(velocity)
                    ball.angular_vel += impact_speed / self.BALL_RADIUS

                    # Move ball outside wall
                    ball.x = closest[0] + normal[0] * self.BALL_RADIUS
                    ball.y = closest[1] + normal[1] * self.BALL_RADIUS

    def check_ball_collision(self, b1: Ball, b2: Ball):
        dx = b2.x - b1.x
        dy = b2.y - b1.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 2 * self.BALL_RADIUS:
            # Collision response
            normal = np.array([dx / dist, dy / dist])

            v1 = np.array([b1.vx, b1.vy])
            v2 = np.array([b2.vx, b2.vy])

            # Elastic collision
            v1_new = v1 - np.dot(v1 - v2, normal) * normal
            v2_new = v2 - np.dot(v2 - v1, -normal) * (-normal)

            b1.vx, b1.vy = v1_new * self.RESTITUTION
            b2.vx, b2.vy = v2_new * self.RESTITUTION

            # Update angular velocities
            relative_vel = np.linalg.norm(v1 - v2)
            b1.angular_vel += relative_vel / (2 * self.BALL_RADIUS)
            b2.angular_vel += relative_vel / (2 * self.BALL_RADIUS)

            # Separate balls
            overlap = 2 * self.BALL_RADIUS - dist
            b1.x -= overlap / 2 * normal[0]
            b1.y -= overlap / 2 * normal[1]
            b2.x += overlap / 2 * normal[0]
            b2.y += overlap / 2 * normal[1]

    def update(self):
        current_time = self.root.tk.call("clock", "milliseconds")
        if self.last_update is None:
            self.last_update = current_time
            self.root.after(16, self.update)
            return

        dt = (current_time - self.last_update) / 1000.0  # Convert to seconds
        self.last_update = current_time

        # Update heptagon rotation
        self.heptagon_angle += self.HEPTAGON_ANGULAR_SPEED * dt
        vertices = self.get_heptagon_vertices()

        # Update ball positions and velocities
        for ball in self.balls:
            ball.vy += self.GRAVITY * dt
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            ball.angle += ball.angular_vel * dt

            # Apply friction
            ball.vx *= self.FRICTION
            ball.vy *= self.FRICTION
            ball.angular_vel *= self.ANGULAR_FRICTION

        # Check collisions
        for i, ball in enumerate(self.balls):
            self.check_wall_collision(ball, vertices)
            for j in range(i + 1, len(self.balls)):
                self.check_ball_collision(ball, self.balls[j])

        # Draw everything
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(
            *[coord for vertex in vertices for coord in vertex],
            outline="black",
            fill="white"
        )

        # Draw balls
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - self.BALL_RADIUS,
                ball.y - self.BALL_RADIUS,
                ball.x + self.BALL_RADIUS,
                ball.y + self.BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )

            # Draw number (rotated)
            text_x = ball.x + self.BALL_RADIUS * 0.5 * math.cos(ball.angle)
            text_y = ball.y + self.BALL_RADIUS * 0.5 * math.sin(ball.angle)
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(self.BALL_RADIUS * 0.8)),
            )

        self.root.after(16, self.update)  # Approximately 60 FPS


if __name__ == "__main__":
    app = BouncingBallsApp()
