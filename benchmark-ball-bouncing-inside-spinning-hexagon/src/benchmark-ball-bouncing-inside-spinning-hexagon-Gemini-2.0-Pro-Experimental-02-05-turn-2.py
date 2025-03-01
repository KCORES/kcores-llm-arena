import tkinter as tk
import math
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Vector:
    x: float
    y: float

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        len = self.length()
        if len == 0:  # Avoid division by zero.
            return Vector(0, 0)
        return Vector(self.x / len, self.y / len)

    def rotate(self, angle):
        """Rotate the vector by a given angle (in radians)."""
        x = self.x * math.cos(angle) - self.y * math.sin(angle)
        y = self.x * math.sin(angle) + self.y * math.cos(angle)
        return Vector(x, y)


@dataclass
class Ball:
    position: Vector
    velocity: Vector
    radius: float
    color: str
    number: int
    rotation: float = 0.0  # Angular position (radians)
    angular_velocity: float = 0.0
    mass: float = 1.0  # Assuming uniform density, mass is proportional to area


class Heptagon:
    def __init__(self, center: Vector, size: float, rotation_speed: float):
        self.center = center
        self.size = size  # Distance from center to a vertex
        self.rotation_speed = rotation_speed  # Radians per second
        self.angle = 0.0  # Current rotation angle

    def get_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.angle + i * 2 * math.pi / 7
            x = self.center.x + self.size * math.cos(angle)
            y = self.center.y + self.size * math.sin(angle)
            vertices.append(Vector(x, y))
        return vertices

    def get_edges(self):
        vertices = self.get_vertices()
        edges = []
        for i in range(7):
            edges.append((vertices[i], vertices[(i + 1) % 7]))
        return edges

    def update(self, dt):
        self.angle += self.rotation_speed * dt
        self.angle %= 2 * math.pi


class Simulation:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(self.root, width=width, height=height, bg="white")
        self.canvas.pack()

        self.ball_radius = 15
        self.num_balls = 20
        self.gravity = Vector(0, 200)  # Downward acceleration
        self.friction = 0.99  # Velocity damping per update
        self.angular_friction = 0.95  # Angular velocity damping
        self.heptagon_size = 250  # Adjust heptagon size as needed
        self.heptagon = Heptagon(
            Vector(width / 2, height / 2), self.heptagon_size, 2 * math.pi / 5
        )  # 360 degrees per 5 seconds
        self.balls: List[Ball] = []
        self.ball_colors = [
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

        self.reset_balls()  # Initialize or reset the balls positions.
        self.last_time = time.time()
        self.heptagon_id = None  # to store canvas heptagon ID for redrawing
        self.ball_ids = []  # to store the canvas ball IDs for redrawing
        self.run()  # Start the main loop.
        self.root.mainloop()

    def reset_balls(self):
        """Resets the simulation, placing balls at the heptagon's center."""
        self.balls = []
        for i in range(self.num_balls):
            ball = Ball(
                position=Vector(self.width / 2, self.height / 2),
                velocity=Vector(0, 0),
                radius=self.ball_radius,
                color=self.ball_colors[i],
                number=i + 1,
            )
            self.balls.append(ball)

    def update_physics(self, dt):

        for ball in self.balls:
            # Apply gravity
            ball.velocity = ball.velocity + self.gravity * dt
            # Apply friction
            ball.velocity = ball.velocity * self.friction
            ball.angular_velocity *= self.angular_friction

            # Update position
            ball.position = ball.position + ball.velocity * dt
            ball.rotation += ball.angular_velocity * dt

        # Ball-ball collision
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.collide_balls(self.balls[i], self.balls[j])

        # Ball-heptagon collision
        for ball in self.balls:
            self.collide_ball_heptagon(ball)

    def collide_balls(self, ball1, ball2):
        collision_vector = ball2.position - ball1.position
        distance = collision_vector.length()
        min_dist = ball1.radius + ball2.radius

        if distance < min_dist:
            # Collision detected!
            normal = collision_vector.normalize()
            overlap = min_dist - distance

            # Move balls apart to avoid sticking
            ball1.position = ball1.position - normal * (overlap / 2)
            ball2.position = ball2.position + normal * (overlap / 2)

            # Calculate relative velocity along the normal
            relative_velocity = ball2.velocity - ball1.velocity
            velocity_along_normal = relative_velocity.dot(normal)

            # Calculate impulse
            impulse = (2 * velocity_along_normal) / (ball1.mass + ball2.mass)

            # Apply impulse
            ball1.velocity = ball1.velocity + normal * impulse * ball2.mass
            ball2.velocity = ball2.velocity - normal * impulse * ball1.mass

            # Calculate relative tangential velocity for friction (spin)
            tangent = Vector(-normal.y, normal.x)  # Perpendicular to normal
            relative_tangent_velocity = (ball2.velocity - ball1.velocity).dot(tangent)

            # Apply angular velocity change (simplified model)
            # Consider the moment of inertia (I = 0.4 * m * r^2 for a solid sphere)
            moment_of_inertia1 = 0.4 * ball1.mass * ball1.radius**2
            moment_of_inertia2 = 0.4 * ball2.mass * ball2.radius**2

            angular_impulse1 = (
                -relative_tangent_velocity
                * ball1.radius
                * ball2.mass
                / (ball1.mass + ball2.mass)
            )
            angular_impulse2 = (
                relative_tangent_velocity
                * ball2.radius
                * ball1.mass
                / (ball1.mass + ball2.mass)
            )

            ball1.angular_velocity += angular_impulse1 / moment_of_inertia1
            ball2.angular_velocity -= angular_impulse2 / moment_of_inertia2

    def collide_ball_heptagon(self, ball):
        edges = self.heptagon.get_edges()

        for edge_start, edge_end in edges:
            # Vector from edge start to edge end
            edge_vector = edge_end - edge_start

            # Vector from edge start to ball center
            start_to_ball = ball.position - edge_start

            # Project start_to_ball onto edge_vector
            projection_length = start_to_ball.dot(edge_vector.normalize())

            # Find the closest point on the edge to the ball
            closest_point = Vector(0, 0)
            if projection_length < 0:
                closest_point = edge_start
            elif projection_length > edge_vector.length():
                closest_point = edge_end
            else:
                closest_point = edge_start + edge_vector.normalize() * projection_length

            # Check for collision
            distance_vector = ball.position - closest_point
            distance = distance_vector.length()

            if distance < ball.radius:

                normal = distance_vector.normalize()
                overlap = ball.radius - distance

                # Move ball to not overlap
                ball.position = ball.position + normal * overlap

                # Reflect velocity
                ball.velocity = (
                    ball.velocity - normal * (2 * ball.velocity.dot(normal)) * 0.8
                )  # Loss of speed during collision.
                if ball.velocity.length() * 0.8 < 20:
                    ball.velocity = Vector(
                        0, 0
                    )  # Avoid very small bouncing by stopping movement.

                # Simple angular velocity change (friction against wall)
                tangent = Vector(-normal.y, normal.x)  # Perpendicular to the normal
                tangential_velocity = ball.velocity.dot(
                    tangent
                )  # Velocity along the tangent

                # The 0.1 factor can be tuned to control the spin effect
                ball.angular_velocity -= (tangential_velocity / ball.radius) * 0.1

    def draw(self):
        # Clear previous drawings
        self.canvas.delete("all")

        # Draw heptagon
        vertices = self.heptagon.get_vertices()
        vertex_coords = [(v.x, v.y) for v in vertices]
        self.heptagon_id = self.canvas.create_polygon(
            vertex_coords, outline="black", fill=""
        )

        # Draw balls
        self.ball_ids = []
        for ball in self.balls:
            x1 = ball.position.x - ball.radius
            y1 = ball.position.y - ball.radius
            x2 = ball.position.x + ball.radius
            y2 = ball.position.y + ball.radius
            ball_id = self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)

            # Draw number text on the ball, rotated
            text_x = ball.position.x
            text_y = ball.position.y

            # Use create_text with rotation
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="black",
                font=("Arial", int(ball.radius * 0.8)),
                angle=math.degrees(ball.rotation),
            )
            self.ball_ids.append(ball_id)

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.heptagon.update(dt)  # Update heptagon rotation
        self.update_physics(dt)  # Update ball positions and velocities
        self.draw()  # Redraw everything

    def run(self):
        self.last_time = time.time()
        while True:  # Use a while loop for continuous updates
            self.update()
            self.root.update()  # Process Tkinter events
            time.sleep(0.01)  # Control the simulation speed


if __name__ == "__main__":
    sim = Simulation(800, 600)  # Adjust window size as necessary
