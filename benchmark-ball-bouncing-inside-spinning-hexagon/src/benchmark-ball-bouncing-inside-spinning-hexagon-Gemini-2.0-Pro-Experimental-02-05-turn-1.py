import tkinter as tk
import math
import numpy as np
import time
from dataclasses import dataclass
from typing import List


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

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalize(self):
        len = self.length()
        return Vector(self.x / len, self.y / len) if len > 0 else Vector(0, 0)

    def rotate(self, angle):  # Rotate counter-clockwise
        rad = math.radians(angle)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        return Vector(self.x * cos_a - self.y * sin_a, self.x * sin_a + self.y * cos_a)

    def angle(self):
        """Returns the angle in radians relative to the positive x-axis."""
        return math.atan2(self.y, self.x)


@dataclass
class Ball:
    position: Vector
    velocity: Vector
    radius: float
    color: str
    number: int
    angular_velocity: float = 0.0  # Radians per second
    mass: float = 1.0


class BouncingBalls:
    def __init__(self, num_balls=20, canvas_width=800, canvas_height=600):
        self.root = tk.Tk()
        self.root.title("Bouncing Balls in a Spinning Heptagon")
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.canvas = tk.Canvas(
            self.root, width=canvas_width, height=canvas_height, bg="white"
        )
        self.canvas.pack()

        self.num_balls = num_balls
        self.balls: List[Ball] = []
        self.ball_radius = 15
        self.gravity = Vector(0, 0.2)  # Adjusted for visual appeal
        self.friction = 0.995  # Air friction
        self.wall_friction = 0.98
        self.ball_ball_friction = 0.97
        self.angular_friction = 0.97

        self.heptagon_radius = 250
        self.heptagon_center = Vector(canvas_width / 2, canvas_height / 2)
        self.rotation_speed = 360 / 5  # Degrees per second
        self.current_angle = 0
        self.start_time = time.time()

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

        self.create_balls()
        self.create_heptagon()
        self.animate()
        self.root.mainloop()

    def create_balls(self):
        for i in range(self.num_balls):
            position = self.heptagon_center  # Balls start at the center
            velocity = Vector(0, 0)  # Initial velocity is zero
            self.balls.append(
                Ball(position, velocity, self.ball_radius, self.colors[i], i + 1)
            )

    def create_heptagon(self):
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.heptagon_id = self.canvas.create_polygon(
            self.heptagon_vertices, outline="black", fill=""
        )
        self.canvas.tag_lower(self.heptagon_id)  # Ensure heptagon is behind balls.

    def calculate_heptagon_vertices(self):
        vertices = []
        for i in range(7):
            angle = math.radians(self.current_angle + i * (360 / 7))
            x = self.heptagon_center.x + self.heptagon_radius * math.cos(angle)
            y = self.heptagon_center.y + self.heptagon_radius * math.sin(angle)
            vertices.append(x)
            vertices.append(y)
        return vertices

    def rotate_heptagon(self):
        elapsed_time = time.time() - self.start_time
        self.current_angle = (elapsed_time * self.rotation_speed) % 360
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        self.canvas.coords(self.heptagon_id, *self.heptagon_vertices)

    def animate(self):
        self.rotate_heptagon()
        self.move_balls()
        self.canvas.after(16, self.animate)  # ~60 FPS

    def move_balls(self):
        for i, ball in enumerate(self.balls):
            # Apply gravity and friction
            ball.velocity = ball.velocity + self.gravity
            ball.velocity = ball.velocity * self.friction
            ball.angular_velocity *= self.angular_friction

            # Update position
            ball.position = ball.position + ball.velocity

            # Ball-wall collision
            self.handle_wall_collision(ball)

            # Ball-ball collisions
            for j in range(i + 1, len(self.balls)):
                self.handle_ball_collision(ball, self.balls[j])

        self.draw_balls()

    def handle_wall_collision(self, ball):
        for i in range(7):
            # Get the vertices of the current edge
            v1_index = (i * 2) % 14
            v2_index = ((i * 2) + 2) % 14

            v1 = Vector(
                self.heptagon_vertices[v1_index], self.heptagon_vertices[v1_index + 1]
            )
            v2 = Vector(
                self.heptagon_vertices[v2_index], self.heptagon_vertices[v2_index + 1]
            )

            # Calculate the edge normal
            edge = v2 - v1
            normal = Vector(-edge.y, edge.x).normalize()

            # Vector from a vertex to the ball's center
            v1_to_ball = ball.position - v1

            # Distance from the ball to the edge
            distance = v1_to_ball.dot(normal)

            # Check for collision
            if distance < ball.radius:

                # Limit bounce height
                bounce_factor = min(
                    1.0, (self.heptagon_radius * 0.8) / ball.velocity.length()
                )  # scale the bounce height, so it will not exceed the limit

                # Reflect the velocity across the normal
                ball.velocity = (
                    (ball.velocity - normal * (2 * ball.velocity.dot(normal)))
                    * bounce_factor
                    * self.wall_friction
                )

                # Move the ball out of the wall
                overlap = ball.radius - distance
                ball.position = ball.position + normal * overlap

                # Calculate angular velocity change due to collision
                tangent_velocity = ball.velocity.dot(edge.normalize())
                ball.angular_velocity += (
                    tangent_velocity / ball.radius
                ) * 0.2  # factor for realistic spin

    def handle_ball_collision(self, ball1, ball2):

        distance_vec = ball2.position - ball1.position
        distance = distance_vec.length()

        if distance < ball1.radius + ball2.radius:
            # Calculate the collision normal
            normal = distance_vec.normalize()

            # Move balls apart to prevent overlap
            overlap = (ball1.radius + ball2.radius) - distance
            ball1.position = ball1.position - normal * (overlap / 2)
            ball2.position = ball2.position + normal * (overlap / 2)

            # Calculate relative velocity
            relative_velocity = ball2.velocity - ball1.velocity

            # Calculate impulse
            impulse = relative_velocity.dot(normal)

            # Apply impulse (simplified, assuming equal mass)
            if impulse < 0:  # Ensure balls are moving towards each other
                ball1.velocity = (
                    ball1.velocity + normal * impulse * self.ball_ball_friction
                )
                ball2.velocity = (
                    ball2.velocity - normal * impulse * self.ball_ball_friction
                )

                # Calculate tangent for angular velocity
                tangent = Vector(-normal.y, normal.x)
                relative_tangent_velocity = relative_velocity.dot(tangent)

                # Apply angular velocity changes (simplified)
                ball1.angular_velocity += (
                    relative_tangent_velocity / ball1.radius
                ) * 0.1  # factors for a visually good result
                ball2.angular_velocity -= (
                    relative_tangent_velocity / ball2.radius
                ) * 0.1

    def draw_balls(self):
        self.canvas.delete("ball")  # Clear previous balls
        for ball in self.balls:

            x1 = ball.position.x - ball.radius
            y1 = ball.position.y - ball.radius
            x2 = ball.position.x + ball.radius
            y2 = ball.position.y + ball.radius
            self.canvas.create_oval(
                x1, y1, x2, y2, fill=ball.color, outline="black", tags="ball"
            )

            # Draw the number (rotate the text according to angular velocity)
            text_angle = ball.angular_velocity * time.time()  # Accumulated rotation

            # Calculate a point on the ball's circumference for text positioning
            text_offset_x = (
                ball.radius * 0.6 * math.cos(text_angle)
            )  # Adjust 0.6 for text position
            text_offset_y = ball.radius * 0.6 * math.sin(text_angle)

            text_x = ball.position.x + text_offset_x
            text_y = ball.position.y + text_offset_y
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", 10),
                tags="ball",
            )


if __name__ == "__main__":
    BouncingBalls()
