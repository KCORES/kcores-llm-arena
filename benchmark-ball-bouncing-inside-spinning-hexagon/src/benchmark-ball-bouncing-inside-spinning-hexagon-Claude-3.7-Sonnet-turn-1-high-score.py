import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import random

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
GRAVITY = 0.2
FRICTION = 0.98
ROTATION_SPEED = 360 / 5  # 360 degrees per 5 seconds
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
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0  # Rotation angle
    angular_velocity: float = 0  # Angular velocity


@dataclass
class Line:
    x1: float
    y1: float
    x2: float
    y2: float

    def length(self) -> float:
        return math.sqrt((self.x2 - self.x1) ** 2 + (self.y2 - self.y1) ** 2)

    def normal(self) -> Tuple[float, float]:
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        length = self.length()
        return (-dy / length, dx / length)


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")

        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.heptagon_angle = 0
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.heptagon_lines = self.calculate_heptagon_lines()
        self.heptagon = self.canvas.create_polygon(
            *[coord for vertex in self.heptagon_vertices for coord in vertex],
            outline="white",
            fill="",
            width=2
        )

        # Create balls
        self.balls = []
        for i in range(20):
            angle = random.uniform(0, 2 * math.pi)
            speed = random.uniform(1, 3)
            self.balls.append(
                Ball(
                    x=CENTER_X,
                    y=CENTER_Y,
                    vx=math.cos(angle) * speed,
                    vy=math.sin(angle) * speed,
                    radius=BALL_RADIUS,
                    color=BALL_COLORS[i],
                    number=i + 1,
                )
            )

        self.ball_objects = []
        self.ball_numbers = []
        for ball in self.balls:
            ball_obj = self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="white",
            )
            number_obj = self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                fill="white",
                font=("Arial", 12, "bold"),
            )
            self.ball_objects.append(ball_obj)
            self.ball_numbers.append(number_obj)

        self.update()

    def calculate_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        for i in range(7):
            angle = self.heptagon_angle + i * (2 * math.pi / 7)
            x = CENTER_X + HEPTAGON_RADIUS * math.cos(angle)
            y = CENTER_Y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def calculate_heptagon_lines(self) -> List[Line]:
        lines = []
        vertices = self.heptagon_vertices
        for i in range(7):
            lines.append(
                Line(
                    vertices[i][0],
                    vertices[i][1],
                    vertices[(i + 1) % 7][0],
                    vertices[(i + 1) % 7][1],
                )
            )
        return lines

    def update_heptagon(self):
        # Update heptagon angle
        self.heptagon_angle += math.radians(ROTATION_SPEED / 60)  # 60 FPS

        # Recalculate vertices and lines
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.heptagon_lines = self.calculate_heptagon_lines()

        # Update heptagon on canvas
        self.canvas.coords(
            self.heptagon,
            *[coord for vertex in self.heptagon_vertices for coord in vertex]
        )

    def check_ball_wall_collision(self, ball: Ball):
        for line in self.heptagon_lines:
            # Vector from line start to ball center
            vx = ball.x - line.x1
            vy = ball.y - line.y1

            # Vector representing the line
            line_vx = line.x2 - line.x1
            line_vy = line.y2 - line.y1
            line_length = math.sqrt(line_vx**2 + line_vy**2)

            # Normalize line vector
            line_vx /= line_length
            line_vy /= line_length

            # Project ball-to-line-start vector onto line
            projection = vx * line_vx + vy * line_vy

            # Find closest point on line to ball
            if projection < 0:
                closest_x, closest_y = line.x1, line.y1
            elif projection > line_length:
                closest_x, closest_y = line.x2, line.y2
            else:
                closest_x = line.x1 + projection * line_vx
                closest_y = line.y1 + projection * line_vy

            # Distance from ball to closest point
            distance = math.sqrt((ball.x - closest_x) ** 2 + (ball.y - closest_y) ** 2)

            # Check collision
            if distance < ball.radius:
                # Calculate normal vector of the line
                nx, ny = line.normal()

                # Calculate penetration depth
                penetration = ball.radius - distance

                # Move ball out of collision
                ball.x += nx * penetration
                ball.y += ny * penetration

                # Calculate reflection
                dot_product = ball.vx * nx + ball.vy * ny
                ball.vx = ball.vx - 2 * dot_product * nx
                ball.vy = ball.vy - 2 * dot_product * ny

                # Apply friction and damping
                ball.vx *= FRICTION
                ball.vy *= FRICTION

                # Update ball rotation based on impact
                tangent_x, tangent_y = ny, -nx  # Tangent is perpendicular to normal
                tangent_velocity = ball.vx * tangent_x + ball.vy * tangent_y
                ball.angular_velocity += (
                    tangent_velocity * 0.1
                )  # Scale factor for visual effect

    def check_ball_ball_collision(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                # Calculate distance between balls
                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.sqrt(dx**2 + dy**2)

                # Check collision
                if distance < ball1.radius + ball2.radius:
                    # Calculate collision normal
                    if distance == 0:  # Avoid division by zero
                        nx, ny = 1, 0
                    else:
                        nx, ny = dx / distance, dy / distance

                    # Calculate relative velocity
                    dvx = ball2.vx - ball1.vx
                    dvy = ball2.vy - ball1.vy

                    # Calculate velocity along normal
                    velocity_along_normal = dvx * nx + dvy * ny

                    # Don't resolve if balls are moving away from each other
                    if velocity_along_normal > 0:
                        continue

                    # Calculate impulse
                    impulse = -(1 + 0.8) * velocity_along_normal  # 0.8 is restitution
                    impulse /= 2  # Equal mass balls

                    # Apply impulse
                    ball1.vx -= impulse * nx
                    ball1.vy -= impulse * ny
                    ball2.vx += impulse * nx
                    ball2.vy += impulse * ny

                    # Resolve penetration
                    penetration = (ball1.radius + ball2.radius - distance) / 2
                    ball1.x -= penetration * nx
                    ball1.y -= penetration * ny
                    ball2.x += penetration * nx
                    ball2.y += penetration * ny

                    # Update angular velocity based on impact
                    tangent_x, tangent_y = -ny, nx  # Tangent is perpendicular to normal
                    tangent_velocity1 = ball1.vx * tangent_x + ball1.vy * tangent_y
                    tangent_velocity2 = ball2.vx * tangent_x + ball2.vy * tangent_y

                    ball1.angular_velocity += (
                        tangent_velocity2 - tangent_velocity1
                    ) * 0.05
                    ball2.angular_velocity += (
                        tangent_velocity1 - tangent_velocity2
                    ) * 0.05

    def update_balls(self):
        for i, ball in enumerate(self.balls):
            # Apply gravity
            ball.vy += GRAVITY

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Update rotation
            ball.angle += ball.angular_velocity

            # Apply angular friction
            ball.angular_velocity *= 0.98

            # Check wall collisions
            self.check_ball_wall_collision(ball)

            # Update ball position on canvas
            self.canvas.coords(
                self.ball_objects[i],
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
            )

            # Update number position and rotation
            self.canvas.coords(self.ball_numbers[i], ball.x, ball.y)

            # Create rotation effect for the number
            rotation_x = ball.radius * 0.5 * math.sin(ball.angle)
            rotation_y = ball.radius * 0.5 * math.cos(ball.angle)
            self.canvas.coords(
                self.ball_numbers[i], ball.x + rotation_x, ball.y + rotation_y
            )

    def update(self):
        self.update_heptagon()
        self.check_ball_ball_collision()
        self.update_balls()
        self.root.after(16, self.update)  # ~60 FPS


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
