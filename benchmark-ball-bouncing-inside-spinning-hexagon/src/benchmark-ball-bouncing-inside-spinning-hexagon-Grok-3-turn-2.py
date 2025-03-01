import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
NUM_BALLS = 20
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.99
HEPTAGON_ROTATION_SPEED = 360 / 5  # degrees per second
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0.0
    angular_velocity: float = 0.0


class HeptagonSimulation:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=800, height=800, bg="white")
        self.canvas.pack()

        # Calculate heptagon size (radius) to contain all balls
        self.heptagon_radius = 300  # Large enough for 20 balls
        self.center_x = 400
        self.center_y = 400
        self.rotation = 0

        # Create balls
        self.balls = []
        for i in range(NUM_BALLS):
            ball = Ball(
                x=self.center_x,
                y=self.center_y,
                vx=0,
                vy=0,
                radius=BALL_RADIUS,
                color=COLORS[i],
                number=i + 1,
            )
            self.balls.append(ball)

        self.last_time = 0
        self.update()

    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        for i in range(7):
            angle = math.radians(360 / 7 * i + self.rotation)
            x = self.center_x + self.heptagon_radius * math.cos(angle)
            y = self.center_y + self.heptagon_radius * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def point_line_distance(
        self, p: Tuple[float, float], v1: Tuple[float, float], v2: Tuple[float, float]
    ) -> float:
        line_vec = np.array(v2) - np.array(v1)
        point_vec = np.array(p) - np.array(v1)
        line_len = np.linalg.norm(line_vec)
        line_unit = line_vec / line_len if line_len > 0 else line_vec
        projection = np.dot(point_vec, line_unit)
        projection = max(0, min(line_len, projection))
        closest = np.array(v1) + line_unit * projection
        return np.linalg.norm(np.array(p) - closest)

    def ball_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]
            dist = self.point_line_distance((ball.x, ball.y), v1, v2)

            if dist <= ball.radius:
                # Calculate normal vector
                wall_vec = np.array(v2) - np.array(v1)
                normal = np.array([-wall_vec[1], wall_vec[0]])
                normal = normal / np.linalg.norm(normal)

                # Position correction
                penetration = ball.radius - dist
                ball.x += normal[0] * penetration
                ball.y += normal[1] * penetration

                # Velocity reflection
                vel = np.array([ball.vx, ball.vy])
                dot = np.dot(vel, normal)
                reflection = vel - 2 * dot * normal
                ball.vx = reflection[0] * 0.8
                ball.vy = reflection[1] * 0.8

                # Add rotation from friction
                ball.angular_velocity += dot * 0.1 / ball.radius

    def ball_ball_collision(self, ball1: Ball, ball2: Ball):
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < ball1.radius + ball2.radius and dist > 0:
            # Normalize collision vector
            nx = dx / dist
            ny = dy / dist

            # Position correction
            overlap = (ball1.radius + ball2.radius - dist) / 2
            ball1.x -= nx * overlap
            ball1.y -= ny * overlap
            ball2.x += nx * overlap
            ball2.y += ny * overlap

            # Velocity update
            v1 = np.array([ball1.vx, ball1.vy])
            v2 = np.array([ball2.vx, ball2.vy])
            normal = np.array([nx, ny])

            relative_vel = v1 - v2
            impulse = 2 * np.dot(relative_vel, normal) / 2  # Mass = 1 for both
            v1 -= impulse * normal
            v2 += impulse * normal

            ball1.vx, ball1.vy = v1
            ball2.vx, ball2.vy = v2

            # Add rotation
            tangent = np.array([-ny, nx])
            tangent_speed = np.dot(relative_vel, tangent)
            ball1.angular_velocity -= tangent_speed * 0.1 / ball1.radius
            ball2.angular_velocity += tangent_speed * 0.1 / ball2.radius

    def update(self):
        self.canvas.delete("all")

        # Update rotation (60 FPS approximation)
        dt = 1 / 60
        self.rotation += HEPTAGON_ROTATION_SPEED * dt
        vertices = self.get_heptagon_vertices()

        # Draw heptagon
        self.canvas.create_polygon(vertices, outline="black", fill="")

        # Update balls
        for i, ball in enumerate(self.balls):
            # Apply gravity and friction
            ball.vy += GRAVITY
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt

            # Update rotation
            ball.angle += ball.angular_velocity * dt
            ball.angular_velocity *= FRICTION

            # Check wall collisions
            self.ball_wall_collision(ball, vertices)

            # Check ball-ball collisions
            for j in range(i + 1, NUM_BALLS):
                self.ball_ball_collision(ball, self.balls[j])

            # Draw ball
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            ball_id = self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)

            # Draw number (rotated)
            num_x = ball.x + math.cos(ball.angle) * ball.radius * 0.5
            num_y = ball.y + math.sin(ball.angle) * ball.radius * 0.5
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                font=("Arial", 12, "bold"),
                fill="white",
            )

        self.master.after(16, self.update)  # ~60 FPS


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    app = HeptagonSimulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
