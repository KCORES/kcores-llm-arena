import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Ball properties
@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    spin: float

# Heptagon properties
class Heptagon:
    def __init__(self, center_x, center_y, radius):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.angle = 0
        self.angular_speed = 2 * math.pi / 5  # 360 degrees per 5 seconds

    def rotate(self, dt):
        self.angle += self.angular_speed * dt
        self.angle %= 2 * math.pi

    def get_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.angle + 2 * math.pi * i / 7
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            vertices.append((x, y))
        return vertices


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.width = 800
        self.height = 800
        self.canvas = tk.Canvas(root, width=self.width, height=self.height, bg="white")
        self.canvas.pack()

        # Colors for balls
        self.colors = ["#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d", "#ec6800",
                       "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", "#e45e32",
                       "#e17b34", "#dd7a56", "#db8449", "#d66a35"]
        self.ball_radius = 10
        self.heptagon_radius = 300
        self.heptagon = Heptagon(self.width // 2, self.height // 2, self.heptagon_radius)
        self.balls = self.create_balls()

        self.gravity = 9.8
        self.friction = 0.99
        self.dt = 0.05

        self.root.after(30, self.update)

    def create_balls(self):
        balls = []
        center_x = self.width // 2
        center_y = self.height // 2
        for i in range(20):
            ball = Ball(id=i + 1, x=center_x, y=center_y, vx=0, vy=0, radius=self.ball_radius,
                        color=self.colors[i], spin=0)
            balls.append(ball)
        return balls

    def update(self):
        # Rotate the heptagon
        self.heptagon.rotate(self.dt)

        # Update ball positions and velocities
        for ball in self.balls:
            ball.vy += self.gravity * self.dt
            ball.x += ball.vx * self.dt
            ball.y += ball.vy * self.dt
            ball.vx *= self.friction
            ball.vy *= self.friction
            ball.spin *= self.friction

            # Check and handle ball - heptagon collisions
            self.handle_ball_heptagon_collision(ball)

        # Check and handle ball - ball collisions
        self.handle_ball_ball_collisions()

        # Clear the canvas and redraw
        self.canvas.delete("all")
        self.draw_heptagon()
        self.draw_balls()

        self.root.after(30, self.update)

    def handle_ball_heptagon_collision(self, ball):
        vertices = self.heptagon.get_vertices()
        for i in range(7):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 7]

            # Calculate the distance from the ball center to the line segment
            numerator = abs((y2 - y1) * ball.x - (x2 - x1) * ball.y + x2 * y1 - y2 * x1)
            denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
            distance = numerator / denominator

            if distance <= ball.radius:
                # Calculate the normal vector of the line segment
                dx = x2 - x1
                dy = y2 - y1
                normal_x = -dy
                normal_y = dx
                length = math.sqrt(normal_x ** 2 + normal_y ** 2)
                normal_x /= length
                normal_y /= length

                # Calculate the dot product of the velocity and the normal vector
                dot_product = ball.vx * normal_x + ball.vy * normal_y

                # Reverse the component of the velocity along the normal vector
                ball.vx -= 2 * dot_product * normal_x
                ball.vy -= 2 * dot_product * normal_y

    def handle_ball_ball_collisions(self):
        num_balls = len(self.balls)
        for i in range(num_balls):
            for j in range(i + 1, num_balls):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.sqrt(dx ** 2 + dy ** 2)

                if distance <= ball1.radius + ball2.radius:
                    # Calculate the normal vector between the two balls
                    normal_x = dx / distance
                    normal_y = dy / distance

                    # Calculate the relative velocity along the normal vector
                    v1n = ball1.vx * normal_x + ball1.vy * normal_y
                    v2n = ball2.vx * normal_x + ball2.vy * normal_y

                    # Calculate the new velocities along the normal vector
                    v1n_new = (v1n * (ball1.radius - ball2.radius) + 2 * ball2.radius * v2n) / (ball1.radius + ball2.radius)
                    v2n_new = (v2n * (ball2.radius - ball1.radius) + 2 * ball1.radius * v1n) / (ball1.radius + ball2.radius)

                    # Update the velocities of the balls
                    ball1.vx += (v1n_new - v1n) * normal_x
                    ball1.vy += (v1n_new - v1n) * normal_y
                    ball2.vx += (v2n_new - v2n) * normal_x
                    ball2.vy += (v2n_new - v2n) * normal_y

                    # Adjust positions to avoid overlap
                    overlap = ball1.radius + ball2.radius - distance
                    ball1.x -= overlap * 0.5 * normal_x
                    ball1.y -= overlap * 0.5 * normal_y
                    ball2.x += overlap * 0.5 * normal_x
                    ball2.y += overlap * 0.5 * normal_y

    def draw_heptagon(self):
        vertices = self.heptagon.get_vertices()
        vertex_coords = []
        for x, y in vertices:
            vertex_coords.extend([x, y])
        self.canvas.create_polygon(vertex_coords, outline="black", fill="")

    def draw_balls(self):
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)
            self.canvas.create_text(ball.x, ball.y, text=str(ball.id))


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
