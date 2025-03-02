import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import random

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
GRAVITY = 0.2
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300
FRICTION = 0.98
ANGULAR_FRICTION = 0.99
RESTITUTION = 0.7  # Coefficient of restitution for bounces
SPIN_FACTOR = 0.3  # How much velocity affects ball rotation
HEPTAGON_ANGULAR_VELOCITY = (
    2 * math.pi / (5 * 60)
)  # 360 degrees per 5 seconds, converted to radians per frame for 60 FPS

# Colors for the balls
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
class Vector:
    x: float
    y: float

    def magnitude(self) -> float:
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self) -> "Vector":
        mag = self.magnitude()
        if mag == 0:
            return Vector(0, 0)
        return Vector(self.x / mag, self.y / mag)

    def dot(self, other: "Vector") -> float:
        return self.x * other.x + self.y * other.y

    def scale(self, scalar: float) -> "Vector":
        return Vector(self.x * scalar, self.y * scalar)

    def add(self, other: "Vector") -> "Vector":
        return Vector(self.x + other.x, self.y + other.y)

    def subtract(self, other: "Vector") -> "Vector":
        return Vector(self.x - other.x, self.y - other.y)


class Ball:
    def __init__(self, number: int, color: str, x: float, y: float):
        self.number = number
        self.color = color
        self.position = Vector(x, y)
        # Random initial velocity
        angle = random.uniform(0, 2 * math.pi)
        speed = random.uniform(1, 3)
        self.velocity = Vector(speed * math.cos(angle), speed * math.sin(angle))
        self.angular_velocity = 0.0
        self.rotation = 0.0  # Rotation angle in radians

    def update(self):
        # Apply gravity
        self.velocity.y += GRAVITY

        # Update position
        self.position.x += self.velocity.x
        self.position.y += self.velocity.y

        # Apply friction
        self.velocity.x *= FRICTION
        self.velocity.y *= FRICTION

        # Update rotation based on angular velocity
        self.rotation += self.angular_velocity

        # Apply angular friction
        self.angular_velocity *= ANGULAR_FRICTION

    def draw(self, canvas):
        x, y = self.position.x, self.position.y

        # Draw the ball
        canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=self.color,
            outline="black",
        )

        # Draw the number with rotation
        text_x = x + BALL_RADIUS * 0.5 * math.sin(self.rotation)
        text_y = y - BALL_RADIUS * 0.5 * math.cos(self.rotation)
        canvas.create_text(
            text_x,
            text_y,
            text=str(self.number),
            font=("Arial", int(BALL_RADIUS * 1.2)),
            fill="white",
        )


class Heptagon:
    def __init__(self, center_x: float, center_y: float, radius: float):
        self.center = Vector(center_x, center_y)
        self.radius = radius
        self.rotation = 0.0  # Rotation angle in radians
        self.vertices = self._calculate_vertices()

    def _calculate_vertices(self) -> List[Vector]:
        vertices = []
        for i in range(7):
            angle = self.rotation + i * 2 * math.pi / 7
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)
            vertices.append(Vector(x, y))
        return vertices

    def update(self):
        self.rotation += HEPTAGON_ANGULAR_VELOCITY
        self.vertices = self._calculate_vertices()

    def draw(self, canvas):
        points = []
        for vertex in self.vertices:
            points.extend([vertex.x, vertex.y])
        canvas.create_polygon(points, outline="black", fill="", width=2)

    def get_edges(self) -> List[Tuple[Vector, Vector]]:
        edges = []
        for i in range(7):
            edges.append((self.vertices[i], self.vertices[(i + 1) % 7]))
        return edges


def distance_point_to_line(
    point: Vector, line_start: Vector, line_end: Vector
) -> Tuple[float, Vector, Vector]:
    """Returns the distance from point to line, and the closest point on the line."""
    line_vec = Vector(line_end.x - line_start.x, line_end.y - line_start.y)
    point_vec = Vector(point.x - line_start.x, point.y - line_start.y)

    line_length = line_vec.magnitude()
    if line_length == 0:
        return point_vec.magnitude(), line_start, Vector(0, 0)

    # Calculate projection
    line_normalized = line_vec.normalize()
    projection_length = point_vec.dot(line_normalized)

    # Find closest point on line segment
    if projection_length < 0:
        closest_point = line_start
        proj = 0
    elif projection_length > line_length:
        closest_point = line_end
        proj = line_length
    else:
        closest_point = Vector(
            line_start.x + line_normalized.x * projection_length,
            line_start.y + line_normalized.y * projection_length,
        )
        proj = projection_length

    # Calculate normal vector (perpendicular to line)
    normal = Vector(-line_normalized.y, line_normalized.x)

    # Calculate distance
    distance_vector = Vector(point.x - closest_point.x, point.y - closest_point.y)
    distance = distance_vector.magnitude()

    return distance, closest_point, normal


def check_ball_wall_collision(ball: Ball, heptagon: Heptagon):
    edges = heptagon.get_edges()

    for edge in edges:
        distance, closest_point, normal = distance_point_to_line(
            ball.position, edge[0], edge[1]
        )

        if distance <= BALL_RADIUS:
            # Calculate overlap
            overlap = BALL_RADIUS - distance

            # Move the ball out of collision
            if distance > 0:  # Avoid division by zero
                ball.position.x += normal.x * overlap
                ball.position.y += normal.y * overlap

            # Calculate reflection
            rel_velocity = Vector(ball.velocity.x, ball.velocity.y)

            # Calculate wall velocity at the point of impact due to rotation
            # The wall's velocity is perpendicular to the line from center to impact point
            impact_point_dir = Vector(
                closest_point.x - heptagon.center.x, closest_point.y - heptagon.center.y
            ).normalize()
            wall_velocity = Vector(
                -impact_point_dir.y * HEPTAGON_ANGULAR_VELOCITY * heptagon.radius,
                impact_point_dir.x * HEPTAGON_ANGULAR_VELOCITY * heptagon.radius,
            )

            # Adjust relative velocity to account for wall movement
            rel_velocity.x -= wall_velocity.x
            rel_velocity.y -= wall_velocity.y

            # Calculate reflection normal and tangential components
            normal_vel = rel_velocity.dot(normal)

            # Only bounce if moving toward the wall
            if normal_vel < 0:
                # Calculate tangential direction
                tangent = Vector(-normal.y, normal.x)
                tangent_vel = rel_velocity.dot(tangent)

                # Update ball velocity with bounce and restitution
                ball.velocity.x = wall_velocity.x + (
                    tangent.x * tangent_vel - normal.x * normal_vel * RESTITUTION
                )
                ball.velocity.y = wall_velocity.y + (
                    tangent.y * tangent_vel - normal.y * normal_vel * RESTITUTION
                )

                # Update ball angular velocity based on tangential velocity component
                ball.angular_velocity += tangent_vel * SPIN_FACTOR


def check_ball_ball_collision(ball1: Ball, ball2: Ball):
    dx = ball2.position.x - ball1.position.x
    dy = ball2.position.y - ball1.position.y
    distance = math.sqrt(dx**2 + dy**2)

    if distance < 2 * BALL_RADIUS:
        # Calculate normal and tangent vectors
        if distance == 0:  # Avoid division by zero
            normal = Vector(1, 0)
        else:
            normal = Vector(dx / distance, dy / distance)
        tangent = Vector(-normal.y, normal.x)

        # Calculate overlap and resolve overlap
        overlap = 2 * BALL_RADIUS - distance
        ball1.position.x -= normal.x * overlap / 2
        ball1.position.y -= normal.y * overlap / 2
        ball2.position.x += normal.x * overlap / 2
        ball2.position.y += normal.y * overlap / 2

        # Dot product of velocity and normal/tangent vectors
        v1n = normal.dot(ball1.velocity)
        v1t = tangent.dot(ball1.velocity)
        v2n = normal.dot(ball2.velocity)
        v2t = tangent.dot(ball2.velocity)

        # Conservation of momentum and kinetic energy
        v1n_new = (v1n * (1 - RESTITUTION) + v2n * (1 + RESTITUTION)) / 2
        v2n_new = (v2n * (1 - RESTITUTION) + v1n * (1 + RESTITUTION)) / 2

        # Convert scalars back to vectors
        v1n_vec = normal.scale(v1n_new)
        v1t_vec = tangent.scale(v1t)
        v2n_vec = normal.scale(v2n_new)
        v2t_vec = tangent.scale(v2t)

        # Set new velocities
        ball1.velocity = v1n_vec.add(v1t_vec)
        ball2.velocity = v2n_vec.add(v2t_vec)

        # Update angular velocities based on tangential contact
        relative_vel_tangent = v1t - v2t
        ball1.angular_velocity -= relative_vel_tangent * SPIN_FACTOR
        ball2.angular_velocity += relative_vel_tangent * SPIN_FACTOR


class BallSimulation:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")

        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.heptagon = Heptagon(CENTER_X, CENTER_Y, HEPTAGON_RADIUS)

        # Create balls
        self.balls = []
        for i in range(20):
            # Start all balls from the center
            ball = Ball(i + 1, BALL_COLORS[i], CENTER_X, CENTER_Y)
            self.balls.append(ball)

        self.update()

    def update(self):
        self.canvas.delete("all")

        # Update heptagon
        self.heptagon.update()

        # Update balls
        for ball in self.balls:
            ball.update()

        # Check for ball-wall collisions
        for ball in self.balls:
            check_ball_wall_collision(ball, self.heptagon)

        # Check for ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                check_ball_ball_collision(self.balls[i], self.balls[j])

        # Draw everything
        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas)

        self.root.after(16, self.update)  # ~60 FPS


if __name__ == "__main__":
    root = tk.Tk()
    app = BallSimulation(root)
    root.mainloop()
