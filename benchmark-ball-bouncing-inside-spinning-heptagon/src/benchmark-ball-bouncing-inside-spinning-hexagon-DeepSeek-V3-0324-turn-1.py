import tkinter as tk
import math
import numpy as np
from typing import List, Tuple, Optional
import random
import time


class Vector2D:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vector2D") -> "Vector2D":
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vector2D":
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: float) -> "Vector2D":
        return Vector2D(self.x / scalar, self.y / scalar)

    def dot(self, other: "Vector2D") -> float:
        return self.x * other.x + self.y * other.y

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self) -> "Vector2D":
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return self / mag

    def rotate(self, angle: float) -> "Vector2D":
        cos_val = math.cos(angle)
        sin_val = math.sin(angle)
        return Vector2D(
            self.x * cos_val - self.y * sin_val, self.x * sin_val + self.y * cos_val
        )

    def to_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)


class Ball:
    def __init__(self, position: Vector2D, radius: float, number: int, color: str):
        self.position = position
        self.velocity = Vector2D(0, 0)
        self.radius = radius
        self.number = number
        self.color = color
        self.angular_velocity = random.uniform(-5, 5)
        self.rotation = 0.0
        self.mass = radius**2  # Mass proportional to area

    def update(self, dt: float, gravity: float, friction: float):
        # Apply gravity
        self.velocity.y += gravity * dt

        # Apply friction
        self.velocity = self.velocity * (1 - friction * dt)

        # Update position
        self.position += self.velocity * dt

        # Update rotation
        self.rotation += self.angular_velocity * dt
        self.angular_velocity *= 1 - friction * dt  # Angular friction

    def collide_with_wall(self, normal: Vector2D, restitution: float):
        velocity_along_normal = self.velocity.dot(normal)

        # Only collide if moving towards the wall
        if velocity_along_normal < 0:
            # Calculate impulse
            j = -(1 + restitution) * velocity_along_normal
            impulse = normal * j

            # Apply impulse
            self.velocity += impulse / self.mass

    def collide_with_ball(self, other: "Ball", restitution: float):
        normal = self.position - other.position
        distance = normal.magnitude()

        if distance == 0 or distance > self.radius + other.radius:
            return

        normal = normal / distance

        # Calculate relative velocity
        relative_velocity = self.velocity - other.velocity
        velocity_along_normal = relative_velocity.dot(normal)

        # Only collide if moving towards each other
        if velocity_along_normal > 0:
            return

        # Calculate impulse scalar
        j = -(1 + restitution) * velocity_along_normal
        j /= 1 / self.mass + 1 / other.mass

        # Apply impulse
        impulse = normal * j
        self.velocity += impulse / self.mass
        other.velocity -= impulse / other.mass

        # Separate balls to prevent sticking
        overlap = (self.radius + other.radius - distance) * 0.5
        correction = normal * overlap
        self.position += correction
        other.position -= correction


class Heptagon:
    def __init__(self, center: Vector2D, radius: float):
        self.center = center
        self.radius = radius
        self.rotation = 0.0
        self.rotation_speed = 2 * math.pi / 5  # 360 degrees per 5 seconds
        self.sides = 7
        self.vertices = self.calculate_vertices()

    def calculate_vertices(self) -> List[Vector2D]:
        vertices = []
        angle_step = 2 * math.pi / self.sides
        for i in range(self.sides):
            angle = self.rotation + i * angle_step
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)
            vertices.append(Vector2D(x, y))
        return vertices

    def update(self, dt: float):
        self.rotation += self.rotation_speed * dt
        self.vertices = self.calculate_vertices()

    def get_edges(self) -> List[Tuple[Vector2D, Vector2D]]:
        edges = []
        for i in range(self.sides):
            edges.append((self.vertices[i], self.vertices[(i + 1) % self.sides]))
        return edges

    def check_collision(self, ball: Ball) -> Optional[Tuple[Vector2D, Vector2D]]:
        closest_edge = None
        min_distance = float("inf")
        closest_point = None

        for edge in self.get_edges():
            point = self.closest_point_on_edge(ball.position, edge)
            distance = (ball.position - point).magnitude()

            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point

        if min_distance < ball.radius:
            edge_vec = closest_edge[1] - closest_edge[0]
            normal = Vector2D(-edge_vec.y, edge_vec.x).normalize()

            # Ensure normal points towards the ball
            if (ball.position - closest_point).dot(normal) < 0:
                normal = normal * -1

            return normal, closest_point

        return None

    @staticmethod
    def closest_point_on_edge(
        point: Vector2D, edge: Tuple[Vector2D, Vector2D]
    ) -> Vector2D:
        a, b = edge
        ab = b - a
        ap = point - a

        ab_mag_sq = ab.x**2 + ab.y**2
        if ab_mag_sq == 0:
            return a

        t = max(0, min(1, ap.dot(ab) / ab_mag_sq))
        return a + ab * t


class BouncingBallsSimulation:
    def __init__(self, width: int = 800, height: int = 800):
        self.width = width
        self.height = height
        self.center = Vector2D(width / 2, height / 2)

        # Physics parameters
        self.gravity = 300.0
        self.friction = 1.0
        self.restitution = 0.8  # Bounciness

        # Create heptagon
        heptagon_radius = min(width, height) * 0.4
        self.heptagon = Heptagon(self.center, heptagon_radius)

        # Create balls
        self.balls = []
        ball_colors = [
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

        ball_radius = heptagon_radius * 0.08
        max_initial_speed = heptagon_radius * 0.5

        for i in range(20):
            # Start all balls at center with random small initial velocity
            velocity = Vector2D(
                random.uniform(-max_initial_speed, max_initial_speed),
                random.uniform(-max_initial_speed, max_initial_speed),
            )

            ball = Ball(
                position=Vector2D(self.center.x, self.center.y),
                radius=ball_radius,
                number=i + 1,
                color=ball_colors[i],
            )
            ball.velocity = velocity
            self.balls.append(ball)

        # Set up tkinter
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg="white")
        self.canvas.pack()

        self.last_time = time.time()
        self.running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.running = False
        self.root.destroy()

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update heptagon
        self.heptagon.update(dt)

        # Update balls
        for ball in self.balls:
            ball.update(dt, self.gravity, self.friction)

            # Check collision with heptagon walls
            collision = self.heptagon.check_collision(ball)
            if collision:
                normal, _ = collision
                ball.collide_with_wall(normal, self.restitution)

            # Keep balls inside heptagon (simple containment)
            # This is a fallback in case collision response isn't perfect
            if (
                ball.position - self.center
            ).magnitude() + ball.radius > self.heptagon.radius:
                direction = (ball.position - self.center).normalize()
                ball.position = self.center + direction * (
                    self.heptagon.radius - ball.radius
                )

        # Check ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.balls[i].collide_with_ball(self.balls[j], self.restitution)

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon
        vertices = []
        for vertex in self.heptagon.vertices:
            vertices.extend(vertex.to_tuple())
        self.canvas.create_polygon(vertices, fill="", outline="black", width=2)

        # Draw balls
        for ball in self.balls:
            x, y = ball.position.to_tuple()
            self.canvas.create_oval(
                x - ball.radius,
                y - ball.radius,
                x + ball.radius,
                y + ball.radius,
                fill=ball.color,
                outline="black",
            )

            # Draw number with rotation
            self.canvas.create_text(
                x,
                y,
                text=str(ball.number),
                fill="black",
                font=("Arial", int(ball.radius * 0.6)),
                angle=math.degrees(ball.rotation),
            )

        self.root.update()

    def run(self):
        while self.running:
            self.update()
            self.draw()

            # Cap at ~60 FPS
            time.sleep(1 / 60)

        self.root.mainloop()


if __name__ == "__main__":
    simulation = BouncingBallsSimulation()
    simulation.run()
