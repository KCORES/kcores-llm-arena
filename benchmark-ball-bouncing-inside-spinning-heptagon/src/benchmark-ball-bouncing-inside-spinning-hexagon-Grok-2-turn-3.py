import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    number: int
    angle: float  # Rotation angle in radians
    angular_velocity: float
    color: str


class HeptagonSimulation:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("Bouncing Balls in Spinning Heptagon")

        # Canvas setup
        self.width = 800
        self.height = 800
        self.canvas = tk.Canvas(
            self.window, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        # Simulation parameters
        self.center_x = self.width / 2
        self.center_y = self.height / 2
        self.heptagon_radius = 300
        self.ball_radius = 15
        self.gravity = 0.5
        self.friction = 0.99
        self.heptagon_angle = 0
        self.heptagon_angular_velocity = 2 * math.pi / 5  # 360 degrees in 5 seconds
        self.dt = 0.1

        # Colors for balls
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

        # Initialize balls
        self.balls = []
        for i in range(20):
            ball = Ball(
                x=self.center_x,
                y=self.center_y,
                vx=0,
                vy=0,
                radius=self.ball_radius,
                number=i + 1,
                angle=0,
                angular_velocity=0,
                color=self.colors[i],
            )
            self.balls.append(ball)

        # Heptagon vertices
        self.update_heptagon_vertices()

        self.running = True
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.animate()

    def update_heptagon_vertices(self):
        self.vertices = []
        for i in range(7):
            angle = self.heptagon_angle + (2 * math.pi * i / 7)
            x = self.center_x + self.heptagon_radius * math.cos(angle)
            y = self.center_y + self.heptagon_radius * math.sin(angle)
            self.vertices.append((x, y))

    def point_line_distance(
        self,
        point: Tuple[float, float],
        line_start: Tuple[float, float],
        line_end: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Calculate distance from point to line and closest point on line"""
        px, py = point
        x1, y1 = line_start
        x2, y2 = line_end

        line_vec = np.array([x2 - x1, y2 - y1])
        point_vec = np.array([px - x1, py - y1])

        line_len = np.linalg.norm(line_vec)
        if line_len == 0:
            return np.linalg.norm(point_vec), (x1, y1)

        line_unit = line_vec / line_len
        t = max(0, min(1, np.dot(point_vec, line_unit)))
        closest = np.array([x1, y1]) + t * line_vec

        return np.linalg.norm(np.array([px, py]) - closest), (closest[0], closest[1])

    def ball_wall_collision(self, ball: Ball):
        for i in range(7):
            start = self.vertices[i]
            end = self.vertices[(i + 1) % 7]
            dist, closest = self.point_line_distance((ball.x, ball.y), start, end)

            if dist <= ball.radius:
                # Normal vector
                wall_vec = np.array([end[0] - start[0], end[1] - start[1]])
                normal = np.array([-wall_vec[1], wall_vec[0]])
                normal = normal / np.linalg.norm(normal)

                # Position correction
                penetration = ball.radius - dist
                ball.x += normal[0] * penetration
                ball.y += normal[1] * penetration

                # Velocity reflection
                v = np.array([ball.vx, ball.vy])
                v_normal = np.dot(v, normal) * normal
                v_tangent = v - v_normal
                ball.vx = (
                    v_tangent[0] - v_normal[0] * 0.8
                )  # 0.8 is restitution coefficient
                ball.vy = v_tangent[1] - v_normal[1] * 0.8

                # Add angular velocity from friction
                tangent_speed = np.linalg.norm(v_tangent)
                ball.angular_velocity += tangent_speed / ball.radius * 0.5

    def ball_ball_collision(self, b1: Ball, b2: Ball):
        dx = b2.x - b1.x
        dy = b2.y - b1.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < b1.radius + b2.radius and dist > 0:
            # Normalize collision normal
            nx = dx / dist
            ny = dy / dist

            # Position correction
            overlap = (b1.radius + b2.radius - dist) / 2
            b1.x -= nx * overlap
            b1.y -= ny * overlap
            b2.x += nx * overlap
            b2.y += ny * overlap

            # Relative velocity
            v1 = np.array([b1.vx, b1.vy])
            v2 = np.array([b2.vx, b2.vy])
            normal = np.array([nx, ny])

            # Collision response
            v1n = np.dot(v1, normal)
            v2n = np.dot(v2, normal)
            v1t = v1 - v1n * normal
            v2t = v2 - v2n * normal

            # Elastic collision (restitution = 0.8)
            v1n_new = v2n * 0.8
            v2n_new = v1n * 0.8

            b1.vx = v1t[0] + v1n_new * normal[0]
            b1.vy = v1t[1] + v1n_new * normal[1]
            b2.vx = v2t[0] + v2n_new * normal[0]
            b2.vy = v2t[1] + v2n_new * normal[1]

            # Add some angular velocity
            tangent_speed = np.linalg.norm(v1t - v2t)
            b1.angular_velocity += tangent_speed / b1.radius * 0.3
            b2.angular_velocity -= tangent_speed / b2.radius * 0.3

    def update_physics(self):
        # Update heptagon rotation
        self.heptagon_angle += self.heptagon_angular_velocity * self.dt
        self.update_heptagon_vertices()

        # Update balls
        for ball in self.balls:
            # Apply gravity
            ball.vy += self.gravity

            # Update position
            ball.x += ball.vx * self.dt
            ball.y += ball.vy * self.dt

            # Apply friction to velocity
            ball.vx *= self.friction
            ball.vy *= self.friction

            # Update rotation
            ball.angle += ball.angular_velocity * self.dt
            ball.angular_velocity *= self.friction

            # Check wall collisions
            self.ball_wall_collision(ball)

        # Check ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.ball_ball_collision(self.balls[i], self.balls[j])

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(self.vertices, outline="black", width=2)

        # Draw balls
        for ball in self.balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            self.canvas.create_oval(x0, y0, x1, y1, fill=ball.color)

            # Draw number rotated with ball
            num_x = ball.x + math.cos(ball.angle) * ball.radius * 0.5
            num_y = ball.y + math.sin(ball.angle) * ball.radius * 0.5
            self.canvas.create_text(num_x, num_y, text=str(ball.number), fill="black")

    def animate(self):
        if self.running:
            self.update_physics()
            self.draw()
            self.window.after(16, self.animate)  # ~60 FPS

    def on_closing(self):
        self.running = False
        self.window.destroy()

    def run(self):
        self.window.mainloop()


if __name__ == "__main__":
    sim = HeptagonSimulation()
    sim.run()
