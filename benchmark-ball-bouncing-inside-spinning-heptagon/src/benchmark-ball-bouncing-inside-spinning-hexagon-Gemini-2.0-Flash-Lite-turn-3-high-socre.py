import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
BALL_COUNT = 20
BALL_RADIUS = 15
HEPTAGON_RADIUS = 250  # Increased to contain all balls
GRAVITY = 0.5
FRICTION = 0.02
ANGULAR_FRICTION = 0.005
HEPTAGON_ROTATION_SPEED = 360 / 5  # Degrees per second
FPS = 60
TIME_STEP = 1 / FPS  # Time between frames
BOUNCE_REDUCTION = 0.6  # Coefficient of restitution

# Colors
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
    spin: float  # Angular velocity
    number: int
    color: str
    radius: float

    def apply_force(self, fx: float, fy: float):
        self.vx += fx * TIME_STEP
        self.vy += fy * TIME_STEP

    def update(self):
        self.apply_force(0, GRAVITY)  # Apply gravity

        self.x += self.vx * TIME_STEP
        self.y += self.vy * TIME_STEP
        self.vx *= 1 - FRICTION
        self.vy *= 1 - FRICTION
        self.spin *= 1 - ANGULAR_FRICTION


def normalize_angle(angle: float) -> float:
    """Normalizes an angle to be within the range [0, 360)."""
    return angle % 360


def heptagon_vertices(
    radius: float, rotation_angle: float
) -> List[Tuple[float, float]]:
    """Calculates the vertices of a rotating heptagon."""
    vertices = []
    for i in range(7):
        angle_degrees = (i * 360 / 7) + rotation_angle
        angle_radians = math.radians(angle_degrees)
        x = CENTER_X + radius * math.cos(angle_radians)
        y = CENTER_Y + radius * math.sin(angle_radians)
        vertices.append((x, y))
    return vertices


def ball_ball_collision(ball1: Ball, ball2: Ball) -> bool:
    """Detects and resolves collisions between two balls."""
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance < ball1.radius + ball2.radius:
        # Collision detected
        overlap = (ball1.radius + ball2.radius) - distance
        nx = dx / distance
        ny = dy / distance

        # Separate the balls
        ball1.x -= 0.5 * overlap * nx
        ball1.y -= 0.5 * overlap * ny
        ball2.x += 0.5 * overlap * nx
        ball2.y += 0.5 * overlap * ny

        # Calculate relative velocity
        relative_vx = ball2.vx - ball1.vx
        relative_vy = ball2.vy - ball1.vy
        dot_product = relative_vx * nx + relative_vy * ny

        # Separate along the normal
        if dot_product < 0:
            # Impulse based collision response
            impulse = -(1 + BOUNCE_REDUCTION) * dot_product / 2
            ball1.vx -= impulse * nx
            ball1.vy -= impulse * ny
            ball2.vx += impulse * nx
            ball2.vy += impulse * ny

        # Apply spin transfer (simplified)
        ball1.spin += 0.1 * (ball2.spin - ball1.spin)
        ball2.spin += 0.1 * (ball1.spin - ball2.spin)

        return True

    return False


def ball_heptagon_collision(
    ball: Ball, heptagon_vertices: List[Tuple[float, float]], rotation_angle: float
):
    """Detects and resolves collisions between a ball and the heptagon walls."""

    for i in range(7):
        v1 = heptagon_vertices[i]
        v2 = heptagon_vertices[(i + 1) % 7]

        # Calculate the distance from the ball's center to the line segment
        dx = v2[0] - v1[0]
        dy = v2[1] - v1[1]
        segment_length_squared = dx * dx + dy * dy

        if segment_length_squared == 0:  # Handle the case where the segment is a point
            closest_x, closest_y = v1
        else:
            t = max(
                0,
                min(
                    1,
                    ((ball.x - v1[0]) * dx + (ball.y - v1[1]) * dy)
                    / segment_length_squared,
                ),
            )
            closest_x = v1[0] + t * dx
            closest_y = v1[1] + t * dy

        distance_x = ball.x - closest_x
        distance_y = ball.y - closest_y
        distance = math.sqrt(distance_x * distance_x + distance_y * distance_y)

        if distance < ball.radius:
            # Collision detected
            overlap = ball.radius - distance
            nx = distance_x / distance
            ny = distance_y / distance

            # Separate the ball
            ball.x += overlap * nx
            ball.y += overlap * ny

            # Calculate the relative velocity
            dot_product = ball.vx * nx + ball.vy * ny

            # Bounce the ball
            if dot_product > 0:
                # Impulse based collision response
                impulse = -(1 + BOUNCE_REDUCTION) * dot_product
                ball.vx -= impulse * nx
                ball.vy -= impulse * ny
                # Limit vertical bounce height
                if ball.vy < 0 and abs(ball.vy) > ball.radius * 0.8:
                    ball.vy = -ball.radius * 0.8

            # Apply spin from the collision (simplified)
            ball.spin += 0.2 * (nx * ball.vy - ny * ball.vx)

            return True

    return False


class BallGame:
    def __init__(self, master):
        self.master = master
        master.title("Bouncing Balls in a Spinning Heptagon")

        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        self.balls: List[Ball] = []
        self.create_balls()
        self.rotation_angle = 0
        self.is_running = True
        self.fps_counter = 0
        self.fps_start_time = 0

        self.update()

    def create_balls(self):
        for i in range(BALL_COUNT):
            # Start balls from the center
            x = CENTER_X
            y = CENTER_Y
            vx = np.random.uniform(-3, 3)
            vy = np.random.uniform(-3, 3)
            spin = np.random.uniform(-1, 1)
            color = COLORS[i % len(COLORS)]
            self.balls.append(Ball(x, y, vx, vy, spin, i + 1, color, BALL_RADIUS))

    def update(self):
        if not self.is_running:
            return

        self.rotation_angle = normalize_angle(
            self.rotation_angle + HEPTAGON_ROTATION_SPEED * TIME_STEP
        )
        heptagon = heptagon_vertices(HEPTAGON_RADIUS, self.rotation_angle)

        # Update ball positions, velocities, and handle collisions
        for ball in self.balls:
            ball.update()
            ball_heptagon_collision(ball, heptagon, self.rotation_angle)

        # Ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball_ball_collision(self.balls[i], self.balls[j])

        self.draw_scene(heptagon)
        self.master.after(
            int(1000 * TIME_STEP), self.update
        )  # Adjust the delay to match FPS

    def draw_scene(self, heptagon_vertices):
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(
            *([coord for vertex in heptagon_vertices for coord in vertex]),
            outline="white",
            fill="",
        )

        # Draw balls
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="white",
            )

            # Draw number on the ball
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), fill="white", font=("Arial", 10)
            )

            # Draw spin indicator (simplified - shows a small line)
            angle = math.radians(ball.spin * 100)  # Convert spin to angle for indicator
            line_x = ball.x + ball.radius * 0.7 * math.cos(angle)
            line_y = ball.y + ball.radius * 0.7 * math.sin(angle)
            self.canvas.create_line(
                ball.x, ball.y, line_x, line_y, fill="white", width=1
            )


if __name__ == "__main__":
    root = tk.Tk()
    game = BallGame(root)
    root.mainloop()
