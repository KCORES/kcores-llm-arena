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
        length = self.length()
        if length != 0:
            return Vector(self.x / length, self.y / length)
        return Vector(0, 0)

    def rotate(self, angle):  # angle in radians
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
    angular_velocity: float = 0.0
    rotation_angle: float = 0.0  # for number rotation


@dataclass
class Segment:
    p1: Vector
    p2: Vector


class Simulation:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.balls: List[Ball] = []
        self.heptagon_radius = 250
        self.heptagon_center = Vector(width / 2, height / 2)
        self.heptagon_segments: List[Segment] = []
        self.gravity = Vector(0, 0.2)  # Adjusted gravity
        self.friction = 0.99  # Added friction
        self.angular_velocity = 2 * math.pi / 5  # 360 degrees per 5 seconds
        self.last_time = time.time()
        self.rotation_angle = 0
        self.ball_radius = 20  # all the balls are the same size
        self.ball_bounce = 0.6  # bounce coeff
        self.collision_damping = 0.8

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

    def create_balls(self):
        for i in range(20):
            position = self.heptagon_center  # all balls from the center
            velocity = Vector(0, 0)  # Initial velocity
            self.balls.append(
                Ball(position, velocity, self.ball_radius, self.colors[i], i + 1)
            )

    def create_heptagon(self):
        self.heptagon_segments.clear()
        for i in range(7):
            angle1 = 2 * math.pi * i / 7 + self.rotation_angle
            angle2 = 2 * math.pi * (i + 1) / 7 + self.rotation_angle
            p1 = Vector(
                self.heptagon_center.x + self.heptagon_radius * math.cos(angle1),
                self.heptagon_center.y + self.heptagon_radius * math.sin(angle1),
            )
            p2 = Vector(
                self.heptagon_center.x + self.heptagon_radius * math.cos(angle2),
                self.heptagon_center.y + self.heptagon_radius * math.sin(angle2),
            )
            self.heptagon_segments.append(Segment(p1, p2))

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update heptagon rotation
        self.rotation_angle += self.angular_velocity * dt
        self.rotation_angle %= 2 * math.pi
        self.create_heptagon()  # Recreate heptagon segments with new angle

        for ball in self.balls:
            # Apply gravity and friction
            ball.velocity = ball.velocity + self.gravity
            ball.velocity = ball.velocity * self.friction
            ball.position = ball.position + ball.velocity * dt * 30  # scaling dt
            ball.angular_velocity *= self.friction
            ball.rotation_angle += ball.angular_velocity * dt * 30  # scaling dt

            # Ball-heptagon collision
            for segment in self.heptagon_segments:
                self.collide_ball_segment(ball, segment)

        # Ball-ball collision
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.collide_balls(self.balls[i], self.balls[j])

    def collide_ball_segment(self, ball: Ball, segment: Segment):
        # Find closest point on segment to ball center
        seg_vec = segment.p2 - segment.p1
        ball_to_p1 = ball.position - segment.p1
        seg_len_sq = seg_vec.dot(seg_vec)

        if seg_len_sq == 0:  # p1 and p2 are the same
            closest_point = segment.p1
        else:
            t = max(0, min(1, ball_to_p1.dot(seg_vec) / seg_len_sq))
            closest_point = segment.p1 + seg_vec * t

        # Check distance
        dist_vec = ball.position - closest_point
        distance = dist_vec.length()

        if distance < ball.radius:
            normal = dist_vec.normalize()
            penetration_depth = ball.radius - distance
            ball.position = ball.position + normal * penetration_depth

            # Collision response with rotation
            relative_velocity = ball.velocity
            velocity_along_normal = relative_velocity.dot(normal)

            if velocity_along_normal < 0:  # Only reflect if moving towards each other
                # collision response
                ball.velocity = ball.velocity - normal * velocity_along_normal * (
                    1 + self.ball_bounce
                )

                # Calculate angular velocity change (simplified)
                tangent_velocity = relative_velocity - normal * velocity_along_normal
                ball.angular_velocity += tangent_velocity.length() / ball.radius * 0.2

    def collide_balls(self, ball1: Ball, ball2: Ball):
        collision_vector = ball2.position - ball1.position
        distance = collision_vector.length()

        if distance < ball1.radius + ball2.radius:
            normal = collision_vector.normalize()
            penetration_depth = (ball1.radius + ball2.radius) - distance

            # Move balls apart
            ball1.position = ball1.position - normal * (penetration_depth / 2)
            ball2.position = ball2.position + normal * (penetration_depth / 2)

            # Collision response
            relative_velocity = ball2.velocity - ball1.velocity
            velocity_along_normal = relative_velocity.dot(normal)

            if velocity_along_normal < 0:
                impulse = -(1 + self.collision_damping) * velocity_along_normal
                impulse /= (
                    1 / ball1.radius + 1 / ball2.radius
                )  # Using radius as a proxy for mass
                ball1.velocity -= normal * (impulse / ball1.radius)
                ball2.velocity += normal * (impulse / ball2.radius)

                # Calculate angular velocity change
                tangent_velocity1 = ball1.velocity - normal * ball1.velocity.dot(normal)
                ball1.angular_velocity += (
                    tangent_velocity1.length() / ball1.radius * 0.1
                )  # friction coeff
                tangent_velocity2 = ball2.velocity - normal * ball2.velocity.dot(normal)
                ball2.angular_velocity -= (
                    tangent_velocity2.length() / ball2.radius * 0.1
                )  # friction coeff


class App:
    def __init__(self, master):
        self.master = master
        master.title("Bouncing Balls in Heptagon")

        self.width = 800
        self.height = 600
        self.canvas = tk.Canvas(
            master, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        self.simulation = Simulation(self.width, self.height)
        self.draw_objects()
        self.animate()

    def draw_objects(self):
        self.canvas.delete("all")

        # Draw heptagon
        for segment in self.simulation.heptagon_segments:
            self.canvas.create_line(
                segment.p1.x, segment.p1.y, segment.p2.x, segment.p2.y, width=2
            )

        # Draw balls
        for ball in self.simulation.balls:
            x1 = ball.position.x - ball.radius
            y1 = ball.position.y - ball.radius
            x2 = ball.position.x + ball.radius
            y2 = ball.position.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)

            # Draw number
            text_offset = Vector(8, 8)  # make the number look in the center
            rotated_pos = (ball.position - text_offset).rotate(
                ball.rotation_angle
            ) + text_offset  # rotate the number position with rotate function
            self.canvas.create_text(
                rotated_pos.x,
                rotated_pos.y,
                text=str(ball.number),
                fill="black",
                font=("Arial", 10),
            )

    def animate(self):
        self.simulation.update()
        self.draw_objects()
        self.master.after(10, self.animate)  # Adjust delay as needed


root = tk.Tk()
app = App(root)
root.mainloop()
