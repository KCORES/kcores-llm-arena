import tkinter as tk
import math
import numpy as np
from typing import List, Tuple

# Constants
NUM_BALLS = 20
HEPTAGON_RADIUS = 400
BALL_RADIUS = 20
GRAVITY = 0.1
FRICTION = 0.99
ROTATION_SPEED = 360 / 5  # degrees per second
COLORS = [
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


class Ball:
    def __init__(self, id: int, color: str):
        self.id = id
        self.color = color
        self.position = np.array([0.0, 0.0], dtype=float)
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.angular_velocity = 0.0
        self.angle = 0.0


class HeptagonBouncingBalls:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.canvas = tk.Canvas(root, width=800, height=800)
        self.canvas.pack()
        self.balls = [Ball(i + 1, COLORS[i]) for i in range(NUM_BALLS)]
        self.heptagon_angle = 0.0
        self.last_time = 0.0
        self.setup_balls()
        self.root.after(10, self.update)

    def setup_balls(self):
        for ball in self.balls:
            ball.position = np.array([0.0, 0.0], dtype=float)
            ball.velocity = np.random.uniform(-3, 3, size=2)
            ball.angular_velocity = np.random.uniform(-0.1, 0.1)

    def update(self):
        current_time = self.root.after_idle(self.root.after, 10, self.update)
        delta_time = (current_time - self.last_time) / 1000.0
        self.last_time = current_time

        self.heptagon_angle += ROTATION_SPEED * delta_time
        self.heptagon_angle %= 360

        for ball in self.balls:
            self.apply_physics(ball, delta_time)
            self.check_collisions(ball)
            self.check_ball_collisions(ball)

        self.draw()

    def apply_physics(self, ball: Ball, delta_time: float):
        ball.velocity[1] += GRAVITY * delta_time
        ball.position += ball.velocity * delta_time
        ball.velocity *= FRICTION
        ball.angle += ball.angular_velocity * delta_time

    def check_collisions(self, ball: Ball):
        # Convert ball position to polar coordinates
        distance = np.linalg.norm(ball.position)
        angle = math.atan2(ball.position[1], ball.position[0])

        # Calculate nearest heptagon wall
        heptagon_angle = (angle - math.radians(self.heptagon_angle)) % (2 * math.pi)
        sector = round(heptagon_angle / (2 * math.pi / 7)) % 7
        wall_angle = math.radians(self.heptagon_angle) + sector * (2 * math.pi / 7)

        # Calculate distance to wall
        wall_distance = HEPTAGON_RADIUS * math.cos(math.pi / 7)

        # Check collision with wall
        if distance + BALL_RADIUS >= wall_distance:
            # Calculate normal vector of the wall
            normal = np.array([math.cos(wall_angle), math.sin(wall_angle)])

            # Reflect velocity
            ball.velocity -= 2 * np.dot(ball.velocity, normal) * normal

            # Adjust position to prevent sticking
            ball.position = normal * (wall_distance - BALL_RADIUS - 1)

            # Apply friction and reduce velocity
            ball.velocity *= 0.8

    def check_ball_collisions(self, ball: Ball):
        for other in self.balls:
            if other.id == ball.id:
                continue

            distance = np.linalg.norm(ball.position - other.position)
            if distance < 2 * BALL_RADIUS:
                # Calculate normal and tangent vectors
                normal = (other.position - ball.position) / distance
                tangent = np.array([-normal[1], normal[0]])

                # Calculate relative velocity
                relative_velocity = ball.velocity - other.velocity

                # Calculate normal and tangent components
                vn = np.dot(relative_velocity, normal) * normal
                vt = np.dot(relative_velocity, tangent) * tangent

                # Apply collision response (elastic collision)
                ball.velocity -= vn
                other.velocity += vn

                # Separate balls to prevent overlap
                overlap = 2 * BALL_RADIUS - distance
                ball.position -= normal * (overlap / 2)
                other.position += normal * (overlap / 2)

    def draw_heptagon(self):
        center = np.array([400, 400])
        points = []
        for i in range(7):
            angle = math.radians(self.heptagon_angle + i * 360 / 7)
            x = center[0] + HEPTAGON_RADIUS * math.cos(angle)
            y = center[1] + HEPTAGON_RADIUS * math.sin(angle)
            points.extend([x, y])
        self.canvas.create_polygon(points, outline="black", fill="")

    def draw_ball(self, ball: Ball):
        x, y = ball.position
        self.canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=ball.color,
            outline="black",
        )
        self.canvas.create_text(x, y, text=str(ball.id), font=("Arial", 10))

    def draw(self):
        self.canvas.delete("all")
        self.draw_heptagon()
        for ball in self.balls:
            self.draw_ball(ball)


if __name__ == "__main__":
    root = tk.Tk()
    app = HeptagonBouncingBalls(root)
    root.mainloop()
