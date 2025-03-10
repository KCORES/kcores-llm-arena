import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List


class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float):
        return Vector(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar: float):
        return Vector(self.x / scalar, self.y / scalar)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)

    def normalized(self):
        mag = self.magnitude()
        if mag == 0:
            return Vector(0, 0)
        return self / mag

    def rotated(self, angle_rad: float):
        cos_angle = math.cos(angle_rad)
        sin_angle = math.sin(angle_rad)
        return Vector(
            self.x * cos_angle - self.y * sin_angle,
            self.x * sin_angle + self.y * cos_angle,
        )


@dataclass
class Ball:
    position: Vector
    velocity: Vector
    radius: float
    color: str
    number: int
    spin: float = 0.0  # Angular velocity in radians per second


class Heptagon:
    def __init__(self, center_x: float, center_y: float, radius: float):
        self.center = Vector(center_x, center_y)
        self.radius = radius
        self.angle_degrees = 0.0
        self.vertices = self._calculate_vertices()

    def _calculate_vertices(self):
        vertices = []
        angle_start_degrees = 0
        for i in range(7):
            angle_radians = math.radians(angle_start_degrees + self.angle_degrees)
            x = self.center.x + self.radius * math.cos(angle_radians)
            y = self.center.y + self.radius * math.sin(angle_radians)
            vertices.append(Vector(x, y))
            angle_start_degrees += 360 / 7
        return vertices

    def rotate(self, angle_degrees_increment: float):
        self.angle_degrees += angle_degrees_increment
        self.vertices = self._calculate_vertices()

    def get_edges(self):
        edges = []
        for i in range(7):
            edges.append((self.vertices[i], self.vertices[(i + 1) % 7]))
        return edges


def check_collision_ball_wall(ball: Ball, edge_p1: Vector, edge_p2: Vector):
    edge_vector = edge_p2 - edge_p1
    ball_to_p1 = ball.position - edge_p1
    edge_normal = Vector(-edge_vector.y, edge_vector.x).normalized()

    distance = ball_to_p1.dot(edge_normal)
    if distance < ball.radius:
        projection = ball_to_p1.dot(edge_vector.normalized())
        if 0 <= projection <= edge_vector.magnitude():
            return edge_normal
    return None


def resolve_collision_ball_wall(
    ball: Ball, normal: Vector, friction: float = 0.1, restitution: float = 0.7
):
    normal_velocity = ball.velocity.dot(normal)
    if normal_velocity < 0:  # Only resolve if moving towards the wall
        ball.velocity = ball.velocity - normal * (1 + restitution) * normal_velocity
        tangent = Vector(-normal.y, normal.x)
        tangent_velocity = ball.velocity.dot(tangent)
        ball.velocity = ball.velocity - tangent * min(
            abs(tangent_velocity), friction * abs(normal_velocity)
        ) * np.sign(tangent_velocity)
        ball.spin += (
            tangent_velocity * 0.1
        )  # Simple spin update based on tangent velocity


def check_collision_ball_ball(ball1: Ball, ball2: Ball):
    distance_vector = ball2.position - ball1.position
    distance = distance_vector.magnitude()
    min_distance = ball1.radius + ball2.radius
    if distance < min_distance:
        return distance_vector.normalized()
    return None


def resolve_collision_ball_ball(
    ball1: Ball, ball2: Ball, normal: Vector, restitution: float = 0.8
):
    relative_velocity = ball2.velocity - ball1.velocity
    normal_velocity = relative_velocity.dot(normal)
    if normal_velocity < 0:
        impulse = (
            normal
            * (-(1 + restitution) * normal_velocity)
            / (1 / 1 + 1 / 1 if 1 else 1)
        )  # Assume unit mass for simplicity
        ball1.velocity = ball1.velocity + impulse
        ball2.velocity = ball2.velocity - impulse


class Simulation:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")
        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(
            self.root, width=self.canvas_width, height=self.canvas_height, bg="white"
        )
        self.canvas.pack()

        self.heptagon_center_x = self.canvas_width / 2
        self.heptagon_center_y = self.canvas_height / 2
        self.heptagon_radius = 200
        self.heptagon_rotation_speed = 360 / 5  # degrees per second
        self.heptagon = Heptagon(
            self.heptagon_center_x, self.heptagon_center_y, self.heptagon_radius
        )

        self.ball_radius = 20
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
        self.balls: List[Ball] = []
        for i in range(20):
            self.balls.append(
                Ball(
                    position=Vector(self.heptagon_center_x, self.heptagon_center_y),
                    velocity=Vector(0, 0),
                    radius=self.ball_radius,
                    color=self.ball_colors[i],
                    number=i + 1,
                )
            )

        self.gravity = Vector(0, 500)  # pixels per second squared
        self.friction_ball = 0.01  # Friction for ball movement
        self.friction_spin = 0.02  # Friction for ball spin
        self.time_step = 0.016  # Roughly 60 FPS
        self.last_time = 0
        self.heptagon_rotation_angle_per_step = (
            self.heptagon_rotation_speed * self.time_step
        )

        self.canvas_heptagon = None
        self.canvas_balls = []
        self.canvas_ball_numbers = []

        self.animate()

    def animate(self):
        current_time = (
            self.root.get_time()
            if hasattr(self.root, "get_time")
            else self.last_time + self.time_step
        )  # For initial run
        delta_time = (
            current_time - self.last_time if self.last_time != 0 else self.time_step
        )  # Avoid large dt at start
        self.last_time = current_time

        self.canvas.delete("all")
        self.update_heptagon(delta_time)
        self.update_balls(delta_time)
        self.draw()

        self.root.after(int(self.time_step * 1000), self.animate)

    def update_heptagon(self, delta_time):
        self.heptagon.rotate(
            self.heptagon_rotation_angle_per_step * (delta_time / self.time_step)
        )  # adjust rotation based on actual delta time

    def update_balls(self, delta_time):
        for ball in self.balls:
            # Apply gravity
            ball.velocity = ball.velocity + self.gravity * delta_time

            # Apply friction to velocity
            if ball.velocity.magnitude() > 0:
                ball.velocity = ball.velocity - ball.velocity.normalized() * min(
                    ball.velocity.magnitude(), self.friction_ball * delta_time * 500
                )  # Scaled friction

            # Apply friction to spin
            if abs(ball.spin) > 0:
                ball.spin -= np.sign(ball.spin) * min(
                    abs(ball.spin), self.friction_spin * delta_time * 10
                )  # Scaled spin friction

            # Move ball
            ball.position = ball.position + ball.velocity * delta_time

        # Ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                normal_vector = check_collision_ball_ball(self.balls[i], self.balls[j])
                if normal_vector:
                    resolve_collision_ball_ball(
                        self.balls[i], self.balls[j], normal_vector
                    )

        # Ball-wall collisions
        edges = self.heptagon.get_edges()
        for ball in self.balls:
            for edge in edges:
                collision_normal = check_collision_ball_wall(ball, edge[0], edge[1])
                if collision_normal:
                    resolve_collision_ball_wall(ball, collision_normal)

        # Bounce height constraint (not strictly limiting to heptagon radius, but bounce damping)
        for ball in self.balls:
            if (
                ball.position.y + ball.radius > self.canvas_height
            ):  # Simple floor collision as a lower bound
                ball.position.y = self.canvas_height - ball.radius
                if ball.velocity.y > 0:
                    ball.velocity.y = -ball.velocity.y * 0.6  # Damping bounce

    def draw(self):
        heptagon_points = []
        for vertex in self.heptagon.vertices:
            heptagon_points.append(vertex.x)
            heptagon_points.append(vertex.y)
        self.canvas_heptagon = self.canvas.create_polygon(
            heptagon_points, outline="black", fill=""
        )

        for i, ball in enumerate(self.balls):
            x0 = ball.position.x - ball.radius
            y0 = ball.position.y - ball.radius
            x1 = ball.position.x + ball.radius
            y1 = ball.position.y + ball.radius
            if i < len(self.canvas_balls):
                self.canvas.coords(self.canvas_balls[i], x0, y0, x1, y1)
            else:
                self.canvas_balls.append(
                    self.canvas.create_oval(
                        x0, y0, x1, y1, fill=ball.color, outline="black"
                    )
                )

            # Draw number - simple rotation by updating text position (not actual text rotation in tkinter easily without external libs)
            number_pos_rotated = Vector(0, ball.radius / 2).rotated(
                ball.spin
            )  # Simple spin visualization by moving number position
            number_x = ball.position.x + number_pos_rotated.x
            number_y = ball.position.y + number_pos_rotated.y

            if i < len(self.canvas_ball_numbers):
                self.canvas.coords(self.canvas_ball_numbers[i], number_x, number_y)
            else:
                self.canvas_ball_numbers.append(
                    self.canvas.create_text(
                        number_x,
                        number_y,
                        text=str(ball.number),
                        fill="white",
                        font=("Arial", int(ball.radius / 2), "bold"),
                    )
                )


if __name__ == "__main__":
    root = tk.Tk()
    if not hasattr(
        root, "get_time"
    ):  # Add get_time for more accurate delta time calculation if tkinter version supports it
        root.get_time = root.tk.getdouble
    sim = Simulation(root)
    root.mainloop()
