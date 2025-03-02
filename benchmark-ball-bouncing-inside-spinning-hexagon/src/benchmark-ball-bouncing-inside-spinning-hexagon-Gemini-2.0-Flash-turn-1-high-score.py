import tkinter as tk
import math
import numpy as np
import dataclasses
import typing
import sys
from typing import List, Tuple


@dataclasses.dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    number: int
    color: str
    spin: float = 0.0  # Rotation angle (degrees)
    spin_speed: float = 0.0  # Angular velocity (degrees per second)
    inertia: float = 0.0  # Moment of inertia

    def __post_init__(self):
        # Set initial spin_speed proportional to velocity, but capped
        self.spin_speed = min(max(self.vx * 5, -360), 360)  # Max 360 deg/s
        self.inertia = 0.5 * self.radius**2  # Assuming uniform density

    def update_position(
        self, dt: float, gravity: float, friction: float, spin_friction: float
    ):
        self.vy += gravity * dt
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Friction affects velocity
        self.vx *= 1 - friction * dt
        self.vy *= 1 - friction * dt

        # Update spin based on spin_speed
        self.spin += self.spin_speed * dt
        self.spin %= 360  # Keep spin within 0-360 degrees

        # Apply rotational friction
        self.spin_speed *= 1 - spin_friction * dt


def create_heptagon_vertices(
    center_x: float, center_y: float, radius: float, angle: float = 0
) -> List[Tuple[float, float]]:
    """Creates vertices of a heptagon."""
    vertices = []
    for i in range(7):
        angle_rad = math.radians(angle + i * 360 / 7)
        x = center_x + radius * math.cos(angle_rad)
        y = center_y + radius * math.sin(angle_rad)
        vertices.append((x, y))
    return vertices


def rotate_point(
    x: float, y: float, angle: float, center_x: float, center_y: float
) -> Tuple[float, float]:
    """Rotates a point around a center."""
    angle_rad = math.radians(angle)
    x -= center_x
    y -= center_y
    new_x = x * math.cos(angle_rad) - y * math.sin(angle_rad)
    new_y = x * math.sin(angle_rad) + y * math.cos(angle_rad)
    return new_x + center_x, new_y + center_y


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculates the distance between two points."""
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def collide_balls(ball1: Ball, ball2: Ball, restitution: float):
    """Handles collision between two balls."""
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    if dist < ball1.radius + ball2.radius:
        # Normalize the collision normal
        normal_x = dx / dist
        normal_y = dy / dist

        # Calculate relative velocity
        rel_vx = ball2.vx - ball1.vx
        rel_vy = ball2.vy - ball1.vy

        # Calculate the dot product of relative velocity and collision normal
        dot_product = rel_vx * normal_x + rel_vy * normal_y

        # If balls are moving away from each other, do nothing
        if dot_product > 0:
            return

        # Calculate impulse scalar
        j = -(1 + restitution) * dot_product
        j /= 1 / ball1.inertia + 1 / ball2.inertia

        # Apply impulse to change velocities
        impulse_x = j * normal_x
        impulse_y = j * normal_y

        ball1.vx -= impulse_x / ball1.inertia
        ball1.vy -= impulse_y / ball1.inertia
        ball2.vx += impulse_x / ball2.inertia
        ball2.vy += impulse_y / ball2.inertia

        # Separate the balls to avoid sticking
        overlap = 0.5 * (ball1.radius + ball2.radius - dist)
        ball1.x -= overlap * normal_x
        ball1.y -= overlap * normal_y
        ball2.x += overlap * normal_x
        ball2.y += overlap * normal_y


def collide_ball_with_edge(
    ball: Ball, v1: Tuple[float, float], v2: Tuple[float, float], restitution: float
):
    """Handles collision between a ball and a heptagon edge."""
    x1, y1 = v1
    x2, y2 = v2

    # Calculate the line normal vector
    edge_x = x2 - x1
    edge_y = y2 - y1
    normal_x = -edge_y
    normal_y = edge_x

    # Normalize the normal
    normal_length = math.sqrt(normal_x**2 + normal_y**2)
    normal_x /= normal_length
    normal_y /= normal_length

    # Project ball center onto the normal
    dist = (ball.x - x1) * normal_x + (ball.y - y1) * normal_y

    # If collision detected
    if dist < ball.radius:
        # Calculate the point of contact
        contact_x = ball.x - dist * normal_x
        contact_y = ball.y - dist * normal_y

        # Check if contact point is within the edge segment
        param = ((contact_x - x1) * edge_x + (contact_y - y1) * edge_y) / (
            edge_x**2 + edge_y**2
        )
        if 0 <= param <= 1:
            # Calculate the relative velocity along the normal
            v_dot_n = ball.vx * normal_x + ball.vy * normal_y

            # Calculate the impulse
            j = -(1 + restitution) * v_dot_n
            ball.vx -= j * normal_x
            ball.vy -= j * normal_y

            # Resolve overlap (avoid sticking)
            overlap = ball.radius - dist
            ball.x += overlap * normal_x
            ball.y += overlap * normal_y


def main():
    width = 800
    height = 600
    center_x = width // 2
    center_y = height // 2
    heptagon_radius = 200
    ball_radius = 15
    num_balls = 20
    gravity = 200  # Adjust as needed
    friction = 0.1  # Adjust as needed
    spin_friction = 0.05  # adjust as needed
    ball_restitution = 0.7  # Adjust as needed
    wall_restitution = 0.7
    rotation_speed = 360 / 5  # Degrees per second

    ball_colors = [
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

    balls: List[Ball] = []
    for i in range(num_balls):
        balls.append(
            Ball(
                center_x,
                center_y,
                np.random.uniform(-50, 50),
                np.random.uniform(-50, 50),
                ball_radius,
                i + 1,
                ball_colors[i % len(ball_colors)],
            )
        )

    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    canvas.pack()

    heptagon_angle = 0.0  # Current rotation angle of the heptagon

    def draw_ball(ball: Ball):
        x0 = ball.x - ball.radius
        y0 = ball.y - ball.radius
        x1 = ball.x + ball.radius
        y1 = ball.y + ball.radius
        canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="")

        # Calculate text position, considering rotation
        text_x = ball.x
        text_y = ball.y

        canvas.create_text(
            text_x,
            text_y,
            text=str(ball.number),
            fill="black",
            font=("Arial", int(ball.radius * 0.8)),
        )

    def draw_heptagon(vertices: List[Tuple[float, float]]):
        canvas.create_polygon(vertices, outline="black", fill="")

    def update():
        nonlocal heptagon_angle
        canvas.delete("all")

        dt = 0.016  # Fixed time step (approx. 60 FPS)

        # Rotate heptagon
        heptagon_angle += rotation_speed * dt
        heptagon_angle %= 360

        # Create rotated heptagon vertices
        heptagon_vertices = create_heptagon_vertices(
            center_x, center_y, heptagon_radius, heptagon_angle
        )

        # Update ball positions and handle collisions
        for ball in balls:
            ball.update_position(dt, gravity, friction, spin_friction)

            # Heptagon edge collision detection
            for i in range(7):
                v1 = heptagon_vertices[i]
                v2 = heptagon_vertices[(i + 1) % 7]
                collide_ball_with_edge(ball, v1, v2, wall_restitution)

        # Ball-ball collision detection
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                collide_balls(balls[i], balls[j], ball_restitution)

        # Draw everything
        draw_heptagon(heptagon_vertices)
        for ball in balls:
            draw_ball(ball)

        root.after(16, update)  # Schedule the next update (approx. 60 FPS)

    update()  # Start the animation loop
    root.mainloop()


if __name__ == "__main__":
    main()
