import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    number: int
    color: str
    spin: float = 0.0  # Rotation in radians
    spin_speed: float = 0.0  # Rotation speed in radians per frame
    moment_of_inertia: float = (
        0.4 * 1 * (radius**2)
    )  # Assuming mass of 1 for simplicity


@dataclass
class Heptagon:
    x: float
    y: float
    radius: float
    rotation: float = 0.0
    rotation_speed: float = (
        2 * math.pi
    ) / 5  # 360 degrees per 5 seconds = 2pi radians / 5 seconds


class BouncingBalls:
    def __init__(
        self, master: tk.Tk, num_balls: int = 20, heptagon_radius: float = 250.0
    ):
        self.master = master
        self.master.title("Bouncing Balls in Spinning Heptagon")

        self.width = 800
        self.height = 600
        self.canvas = tk.Canvas(
            self.master, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        self.num_balls = num_balls
        self.ball_radius = 15.0
        self.gravity = 0.5
        self.friction = 0.01
        self.collision_loss = 0.8  # Coefficient of restitution
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
        self.heptagon = Heptagon(
            x=self.width / 2, y=self.height / 2, radius=heptagon_radius
        )
        self.heptagon_points = self.calculate_heptagon_points()  # Cache these

        self.initialize_balls()

        self.animate()

    def initialize_balls(self):
        for i in range(self.num_balls):
            self.balls.append(
                Ball(
                    x=self.heptagon.x,  # Start at the center
                    y=self.heptagon.y,
                    vx=np.random.uniform(-5, 5),
                    vy=np.random.uniform(-5, 5),
                    radius=self.ball_radius,
                    number=i + 1,
                    color=self.ball_colors[i % len(self.ball_colors)],
                )
            )

    def calculate_heptagon_points(self) -> List[Tuple[float, float]]:
        points = []
        for i in range(7):
            angle = self.heptagon.rotation + (2 * math.pi / 7) * i
            x = self.heptagon.x + self.heptagon.radius * math.cos(angle)
            y = self.heptagon.y + self.heptagon.radius * math.sin(angle)
            points.append((x, y))
        return points

    def draw_heptagon(self):
        points = self.calculate_heptagon_points()
        self.canvas.create_polygon(points, outline="black", fill="", width=2)

    def draw_ball(self, ball: Ball):
        x0 = ball.x - ball.radius
        y0 = ball.y - ball.radius
        x1 = ball.x + ball.radius
        y1 = ball.y + ball.radius
        self.canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")

        # Draw the number, rotate the number based on the spin
        angle = ball.spin
        text_x = ball.x + ball.radius / 2 * math.cos(angle)
        text_y = ball.y + ball.radius / 2 * math.sin(angle)

        self.canvas.create_text(
            ball.x,
            ball.y,
            text=str(ball.number),
            fill="white",
            font=("Arial", int(ball.radius * 0.7)),
        )

    def update_ball_position(self, ball: Ball):
        ball.vy += self.gravity
        ball.x += ball.vx
        ball.y += ball.vy

        # Apply friction
        ball.vx *= 1 - self.friction
        ball.vy *= 1 - self.friction
        ball.spin_speed *= 1 - self.friction

        # Update ball rotation
        ball.spin += ball.spin_speed

    def check_heptagon_collision(self, ball: Ball):
        points = self.calculate_heptagon_points()
        for i in range(7):
            p1 = points[i]
            p2 = points[(i + 1) % 7]
            distance = self.distance_to_segment(ball.x, ball.y, p1, p2)

            if distance <= ball.radius:
                # Collision detected
                normal_x = p2[1] - p1[1]
                normal_y = p1[0] - p2[0]
                norm = math.sqrt(normal_x**2 + normal_y**2)
                normal_x /= norm
                normal_y /= norm

                # Calculate the component of the ball's velocity along the normal
                dot_product = ball.vx * normal_x + ball.vy * normal_y

                # Reverse the velocity component along the normal
                ball.vx -= (1 + self.collision_loss) * dot_product * normal_x
                ball.vy -= (1 + self.collision_loss) * dot_product * normal_y

                # Angular velocity (spin) changes based on the collision
                tangent_x = -normal_y
                tangent_y = normal_x
                tangent_velocity = ball.vx * tangent_x + ball.vy * tangent_y

                ball.spin_speed += (
                    ball.radius * tangent_velocity
                ) / ball.moment_of_inertia

                # Move the ball out of the wall
                overlap = ball.radius - distance
                ball.x += normal_x * overlap
                ball.y += normal_y * overlap

    def distance_to_segment(
        self, px: float, py: float, p1: Tuple[float, float], p2: Tuple[float, float]
    ) -> float:
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        if dx == 0 and dy == 0:
            # It's a point, not a line segment
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)

        t = max(0, min(1, t))  # Clamp t to be between 0 and 1

        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    def check_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.sqrt(dx**2 + dy**2)

                if distance < ball1.radius + ball2.radius:
                    # Collision detected
                    normal_x = dx / distance
                    normal_y = dy / distance

                    # Calculate relative velocity
                    relative_vx = ball2.vx - ball1.vx
                    relative_vy = ball2.vy - ball1.vy

                    # Calculate dot product of relative velocity and normal
                    dot_product = relative_vx * normal_x + relative_vy * normal_y

                    if dot_product < 0:
                        # Impulse along the normal
                        impulse = (
                            (1 + self.collision_loss) * dot_product / (1 / 1 + 1 / 1)
                        )  # simplified since masses = 1
                        impulse_x = impulse * normal_x
                        impulse_y = impulse * normal_y

                        # Update velocities
                        ball1.vx += impulse_x
                        ball1.vy += impulse_y
                        ball2.vx -= impulse_x
                        ball2.vy -= impulse_y

                        # Move balls out of collision (penetration resolution)
                        overlap = ball1.radius + ball2.radius - distance
                        ball1.x -= normal_x * overlap * 0.5
                        ball1.y -= normal_y * overlap * 0.5
                        ball2.x += normal_x * overlap * 0.5
                        ball2.y += normal_y * overlap * 0.5

                        # Rotational Change due to Collision
                        tangent_x = -normal_y
                        tangent_y = normal_x
                        impulse_magnitude = math.sqrt(impulse_x**2 + impulse_y**2)
                        torque1 = impulse_magnitude * ball1.radius
                        torque2 = impulse_magnitude * ball2.radius

                        ball1.spin_speed -= torque1 / ball1.moment_of_inertia
                        ball2.spin_speed += torque2 / ball2.moment_of_inertia

    def animate(self):
        self.canvas.delete("all")  # Clear the canvas

        self.heptagon.rotation += (
            self.heptagon.rotation_speed / 60
        )  # Approximate framerate 60fps
        self.heptagon_points = self.calculate_heptagon_points()  # Recalculate points
        self.draw_heptagon()

        for ball in self.balls:
            self.update_ball_position(ball)
            self.check_heptagon_collision(ball)
            self.draw_ball(ball)

        self.check_ball_collisions()

        self.master.after(16, self.animate)  # Approximately 60 frames per second


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBalls(root)
    root.mainloop()
