import tkinter as tk
import math
import numpy as np
import time
from dataclasses import dataclass
from typing import List, Tuple


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
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    angle: float = 0  # Rotation angle
    angular_velocity: float = 2 * math.pi / 5  # 360 degrees per 5 seconds


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")

        # Constants
        self.WIDTH = 800
        self.HEIGHT = 600
        self.GRAVITY = 9.8 * 30  # Gravity constant (scaled)
        self.FRICTION = 0.98  # Friction coefficient
        self.RESTITUTION = 0.8  # Coefficient of restitution
        self.ANGULAR_FRICTION = 0.98  # Angular friction

        # Colors for the balls
        self.COLORS = [
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

        # Create canvas
        self.canvas = tk.Canvas(root, width=self.WIDTH, height=self.HEIGHT, bg="black")
        self.canvas.pack()

        # Create heptagon
        heptagon_radius = min(self.WIDTH, self.HEIGHT) * 0.4
        self.heptagon = Heptagon(
            center_x=self.WIDTH / 2, center_y=self.HEIGHT / 2, radius=heptagon_radius
        )

        # Create balls
        self.balls = []
        ball_radius = heptagon_radius / 10
        for i in range(20):
            # All balls start from the center with random velocities
            angle = 2 * math.pi * i / 20
            speed = np.random.uniform(50, 100)
            self.balls.append(
                Ball(
                    x=self.heptagon.center_x,
                    y=self.heptagon.center_y,
                    vx=speed * math.cos(angle),
                    vy=speed * math.sin(angle),
                    radius=ball_radius,
                    color=self.COLORS[i],
                    number=i + 1,
                )
            )

        # Start animation
        self.last_time = time.time()
        self.update()

    def get_heptagon_vertices(self):
        """Get the vertices of the heptagon based on current rotation"""
        vertices = []
        for i in range(7):
            angle = self.heptagon.angle + 2 * math.pi * i / 7
            x = self.heptagon.center_x + self.heptagon.radius * math.cos(angle)
            y = self.heptagon.center_y + self.heptagon.radius * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def draw_heptagon(self):
        """Draw the heptagon on the canvas"""
        vertices = self.get_heptagon_vertices()
        # Convert vertices to flat list for create_polygon
        flat_vertices = [coord for vertex in vertices for coord in vertex]
        self.canvas.create_polygon(flat_vertices, outline="white", fill="", width=2)

    def draw_balls(self):
        """Draw all balls on the canvas"""
        for ball in self.balls:
            # Draw the ball
            self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="white",
            )

            # Draw the number (rotated to show spin)
            text_x = ball.x + 0.5 * ball.radius * math.cos(ball.angle)
            text_y = ball.y + 0.5 * ball.radius * math.sin(ball.angle)
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(ball.radius * 0.8)),
            )

    def check_ball_wall_collision(self, ball, dt):
        """Check and handle collision between a ball and the heptagon walls"""
        vertices = self.get_heptagon_vertices()

        # For each edge of the heptagon
        for i in range(7):
            p1 = vertices[i]
            p2 = vertices[(i + 1) % 7]

            # Calculate the closest point on the line segment to the ball
            closest_point = self.closest_point_on_segment(
                ball.x, ball.y, p1[0], p1[1], p2[0], p2[1]
            )

            # Calculate distance from ball to closest point
            dx = ball.x - closest_point[0]
            dy = ball.y - closest_point[1]
            distance = math.sqrt(dx * dx + dy * dy)

            # If collision detected
            if distance < ball.radius:
                # Calculate normal vector of the wall
                wall_dx = p2[0] - p1[0]
                wall_dy = p2[1] - p1[1]
                wall_length = math.sqrt(wall_dx * wall_dx + wall_dy * wall_dy)

                # Normal vector (perpendicular to the wall)
                nx = -wall_dy / wall_length
                ny = wall_dx / wall_length

                # Make sure normal points toward the ball
                if dx * nx + dy * ny < 0:
                    nx, ny = -nx, -ny

                # Move ball outside the wall
                penetration = ball.radius - distance
                ball.x += nx * penetration
                ball.y += ny * penetration

                # Calculate relative velocity
                dot_product = ball.vx * nx + ball.vy * ny

                # Apply impulse
                impulse = -(1 + self.RESTITUTION) * dot_product
                ball.vx += impulse * nx
                ball.vy += impulse * ny

                # Apply angular velocity change based on tangential velocity
                tangent_x = -ny
                tangent_y = nx
                tangent_vel = ball.vx * tangent_x + ball.vy * tangent_y
                ball.angular_velocity += tangent_vel / ball.radius

    def check_ball_ball_collision(self, ball1, ball2):
        """Check and handle collision between two balls"""
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.sqrt(dx * dx + dy * dy)

        # If collision detected
        if distance < ball1.radius + ball2.radius:
            # Normal vector
            if distance == 0:  # Avoid division by zero
                nx, ny = 1, 0
            else:
                nx = dx / distance
                ny = dy / distance

            # Relative velocity
            dvx = ball2.vx - ball1.vx
            dvy = ball2.vy - ball1.vy

            # Velocity along normal
            normal_vel = dvx * nx + dvy * ny

            # If balls are moving toward each other
            if normal_vel < 0:
                # Coefficient of restitution
                e = self.RESTITUTION

                # Impulse
                j = -(1 + e) * normal_vel / 2

                # Apply impulse
                ball1.vx -= j * nx
                ball1.vy -= j * ny
                ball2.vx += j * nx
                ball2.vy += j * ny

                # Apply angular velocity changes based on tangential velocity
                tangent_x = -ny
                tangent_y = nx

                tangent_vel1 = ball1.vx * tangent_x + ball1.vy * tangent_y
                tangent_vel2 = ball2.vx * tangent_x + ball2.vy * tangent_y

                ball1.angular_velocity += (tangent_vel2 - tangent_vel1) / ball1.radius
                ball2.angular_velocity += (tangent_vel1 - tangent_vel2) / ball2.radius

            # Move balls apart to prevent sticking
            overlap = (ball1.radius + ball2.radius - distance) / 2
            ball1.x -= overlap * nx
            ball1.y -= overlap * ny
            ball2.x += overlap * nx
            ball2.y += overlap * ny

    def closest_point_on_segment(self, px, py, x1, y1, x2, y2):
        """Find the closest point on a line segment to a point"""
        # Vector from (x1,y1) to (x2,y2)
        dx = x2 - x1
        dy = y2 - y1

        # If the line segment is just a point
        if dx == 0 and dy == 0:
            return (x1, y1)

        # Calculate projection of point onto line
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

        # Clamp t to [0,1] for line segment
        t = max(0, min(1, t))

        # Calculate closest point
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return (closest_x, closest_y)

    def update(self):
        """Update the simulation"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Limit dt to prevent instability
        dt = min(dt, 0.05)

        # Update heptagon rotation
        self.heptagon.angle += self.heptagon.angular_velocity * dt

        # Update ball positions and velocities
        for ball in self.balls:
            # Apply gravity
            ball.vy += self.GRAVITY * dt

            # Update position
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt

            # Update rotation
            ball.angle += ball.angular_velocity * dt

            # Apply friction
            ball.vx *= self.FRICTION
            ball.vy *= self.FRICTION
            ball.angular_velocity *= self.ANGULAR_FRICTION

            # Check for collision with heptagon walls
            self.check_ball_wall_collision(ball, dt)

        # Check for collisions between balls
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.check_ball_ball_collision(self.balls[i], self.balls[j])

        # Clear canvas and redraw
        self.canvas.delete("all")
        self.draw_heptagon()
        self.draw_balls()

        # Schedule next update
        self.root.after(16, self.update)  # ~60 FPS


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
