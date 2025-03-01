import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys
import random

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
GRAVITY = 0.2
FRICTION = 0.98
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300
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
ROTATION_SPEED = 2 * math.pi / 5  # radians per second
BOUNCE_DAMPING = 0.8
ANGULAR_FRICTION = 0.95


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
    angular_velocity: float = 0

    def update(self, dt: float):
        # Apply gravity
        self.vy += GRAVITY

        # Update position
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Apply friction
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.angular_velocity *= ANGULAR_FRICTION

        # Update angle for rotation
        self.angle += self.angular_velocity * dt


class BounceSimulation:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")

        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.heptagon_angle = 0
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.heptagon_id = self.canvas.create_polygon(
            *self.flatten_vertices(self.heptagon_vertices),
            outline="white",
            fill="",
            width=2
        )

        # Initialize balls at the center with random velocities
        self.balls = []
        for i in range(20):
            angle = 2 * math.pi * random.random()
            speed = 2 + 3 * random.random()
            self.balls.append(
                Ball(
                    x=CENTER_X,
                    y=CENTER_Y,
                    vx=speed * math.cos(angle),
                    vy=speed * math.sin(angle),
                    radius=BALL_RADIUS,
                    color=BALL_COLORS[i],
                    number=i + 1,
                    angular_velocity=0,
                )
            )

        self.ball_ids = [None] * len(self.balls)
        self.text_ids = [None] * len(self.balls)

        self.last_time = self.get_time()
        self.update()

    def get_time(self):
        return self.root.winfo_screenticks() / 1000.0

    def calculate_heptagon_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.heptagon_angle + 2 * math.pi * i / 7
            x = CENTER_X + HEPTAGON_RADIUS * math.cos(angle)
            y = CENTER_Y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def flatten_vertices(self, vertices):
        return [coord for vertex in vertices for coord in vertex]

    def update(self):
        current_time = self.get_time()
        dt = min(
            current_time - self.last_time, 0.05
        )  # Cap at 50ms to prevent large jumps
        self.last_time = current_time

        # Update heptagon rotation
        self.heptagon_angle += ROTATION_SPEED * dt
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.canvas.coords(
            self.heptagon_id, *self.flatten_vertices(self.heptagon_vertices)
        )

        # Update balls
        for ball in self.balls:
            ball.update(dt)

        # Check for collisions with walls
        self.check_wall_collisions()

        # Check for collisions between balls
        self.check_ball_collisions()

        # Draw balls
        self.draw_balls()

        self.root.after(16, self.update)  # ~60 FPS

    def check_wall_collisions(self):
        for ball in self.balls:
            # For each ball, check collision with each wall segment
            for i in range(7):
                p1 = self.heptagon_vertices[i]
                p2 = self.heptagon_vertices[(i + 1) % 7]

                # Check if ball collides with this wall segment
                collision = self.check_ball_line_collision(ball, p1, p2)
                if collision:
                    # Handle collision
                    normal_x, normal_y, penetration = collision

                    # Move ball out of the wall
                    ball.x += normal_x * penetration
                    ball.y += normal_y * penetration

                    # Calculate impact velocity along the normal
                    impact_velocity = ball.vx * normal_x + ball.vy * normal_y

                    # Only bounce if the ball is moving into the wall
                    if impact_velocity < 0:
                        # Apply bounce with damping
                        ball.vx -= (1 + BOUNCE_DAMPING) * impact_velocity * normal_x
                        ball.vy -= (1 + BOUNCE_DAMPING) * impact_velocity * normal_y

                        # Apply angular velocity based on impact and tangent
                        tangent_x, tangent_y = -normal_y, normal_x
                        tangential_velocity = ball.vx * tangent_x + ball.vy * tangent_y
                        ball.angular_velocity += tangential_velocity / ball.radius

    def check_ball_line_collision(self, ball, p1, p2):
        # Calculate vector from p1 to p2
        line_x = p2[0] - p1[0]
        line_y = p2[1] - p1[1]
        line_length = math.sqrt(line_x**2 + line_y**2)

        # Normalize
        if line_length > 0:
            line_x /= line_length
            line_y /= line_length

        # Vector from p1 to ball center
        ball_x = ball.x - p1[0]
        ball_y = ball.y - p1[1]

        # Project ball onto line
        projection = ball_x * line_x + ball_y * line_y

        # Clamp projection to line segment
        projection = max(0, min(line_length, projection))

        # Find closest point on the line segment
        closest_x = p1[0] + projection * line_x
        closest_y = p1[1] + projection * line_y

        # Vector from closest point to ball center
        distance_x = ball.x - closest_x
        distance_y = ball.y - closest_y
        distance = math.sqrt(distance_x**2 + distance_y**2)

        # Check if the ball is colliding
        if distance < ball.radius:
            # Calculate normal vector
            if distance > 0:
                normal_x = distance_x / distance
                normal_y = distance_y / distance
            else:
                # If ball center is exactly on the line, use perpendicular to line
                normal_x, normal_y = -line_y, line_x

            penetration = ball.radius - distance
            return (normal_x, normal_y, penetration)

        return None

    def check_ball_collisions(self):
        num_balls = len(self.balls)
        for i in range(num_balls):
            for j in range(i + 1, num_balls):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.sqrt(dx**2 + dy**2)
                min_distance = ball1.radius + ball2.radius

                if distance < min_distance:
                    # Collision detected
                    if distance == 0:
                        # Avoid division by zero
                        nx, ny = 1, 0
                        penetration = min_distance
                    else:
                        # Normal vector
                        nx = dx / distance
                        ny = dy / distance
                        penetration = min_distance - distance

                    # Move balls apart
                    ball1.x -= nx * penetration / 2
                    ball1.y -= ny * penetration / 2
                    ball2.x += nx * penetration / 2
                    ball2.y += ny * penetration / 2

                    # Calculate relative velocity along normal
                    rvx = ball2.vx - ball1.vx
                    rvy = ball2.vy - ball1.vy
                    rv_normal = rvx * nx + rvy * ny

                    # Do not resolve if velocities are separating
                    if rv_normal < 0:
                        # Impact speed
                        restitution = BOUNCE_DAMPING
                        j_impulse = -(1 + restitution) * rv_normal
                        j_impulse /= 2  # Equal mass assumption

                        # Apply impulse
                        ball1.vx -= j_impulse * nx
                        ball1.y -= j_impulse * ny
                        ball2.vx += j_impulse * nx
                        ball2.vy += j_impulse * ny

                        # Apply angular velocity changes
                        # Tangent vector
                        tx, ty = -ny, nx
                        rv_tangent = rvx * tx + rvy * ty

                        ball1.angular_velocity += rv_tangent / ball1.radius
                        ball2.angular_velocity -= rv_tangent / ball2.radius

    def draw_balls(self):
        for i, ball in enumerate(self.balls):
            # Delete old ball and text
            if self.ball_ids[i] is not None:
                self.canvas.delete(self.ball_ids[i])
            if self.text_ids[i] is not None:
                self.canvas.delete(self.text_ids[i])

            # Create new ball
            self.ball_ids[i] = self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="white",
            )

            # Create number with rotation
            # We'll use a rotation transformation for the text by calculating the rotated position
            angle_rad = ball.angle
            text_x = ball.x + (ball.radius * 0.5) * math.sin(angle_rad)
            text_y = ball.y - (ball.radius * 0.5) * math.cos(angle_rad)

            self.text_ids[i] = self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(ball.radius * 0.8)),
            )


def main():
    root = tk.Tk()
    app = BounceSimulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
