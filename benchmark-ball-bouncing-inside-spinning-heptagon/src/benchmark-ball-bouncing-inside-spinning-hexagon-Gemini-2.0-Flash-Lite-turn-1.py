import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Union

# Constants
WIDTH, HEIGHT = 800, 600
BALL_RADIUS = 10
NUM_BALLS = 20
HEPTAGON_RADIUS = 200
GRAVITY = 0.2
FRICTION = 0.97  # Slows balls down on ground/collision
ANGULAR_FRICTION = 0.98  # Slows spinning
BOUNCE_REDUCTION = 0.6  # Bounce height reduction after impact

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
    color: str
    number: int

    def update(self):
        self.vx *= FRICTION
        self.vy *= FRICTION
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.spin *= ANGULAR_FRICTION


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def resolve_ball_collision(ball1: Ball, ball2: Ball):
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    if dist < 2 * BALL_RADIUS:
        # Calculate collision normal
        nx = (ball2.x - ball1.x) / dist
        ny = (ball2.y - ball1.y) / dist

        # Calculate relative velocity
        v1x = ball1.vx
        v1y = ball1.vy
        v2x = ball2.vx
        v2y = ball2.vy
        rel_vx = v1x - v2x
        rel_vy = v1y - v2y

        # Calculate collision dot product
        dot_product = rel_vx * nx + rel_vy * ny

        # Separate balls to prevent overlapping
        overlap = (2 * BALL_RADIUS - dist) / 2
        ball1.x -= overlap * nx
        ball1.y -= overlap * ny
        ball2.x += overlap * nx
        ball2.y += overlap * ny

        # Adjust velocities based on dot product
        if dot_product > 0:  # Balls are moving away
            return

        # Apply impulse
        impulse = (
            1.1 * dot_product
        )  # Apply some more impulse in case of small dot_product to avoid sticking.
        ball1.vx -= impulse * nx
        ball1.vy -= impulse * ny
        ball2.vx += impulse * nx
        ball2.vy += impulse * ny

        # Simulate spin transfer
        # Adjust spin based on the collision
        spin_transfer = 0.01 * (
            ball1.number - ball2.number
        )  # Simple spin transfer model
        ball1.spin -= spin_transfer
        ball2.spin += spin_transfer


def rotate_point(
    x: float, y: float, angle: float, center_x: float, center_y: float
) -> Tuple[float, float]:
    rad = math.radians(angle)
    new_x = center_x + (x - center_x) * math.cos(rad) - (y - center_y) * math.sin(rad)
    new_y = center_y + (x - center_x) * math.sin(rad) + (y - center_y) * math.cos(rad)
    return new_x, new_y


class Game:
    def __init__(self, master: tk.Tk):
        self.master = master
        master.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.center_x, self.center_y = WIDTH // 2, HEIGHT // 2
        self.balls: List[Ball] = []
        self.heptagon_vertices: List[Tuple[float, float]] = []
        self.rotation_angle = 0
        self.heptagon_sides = 7

        self.create_balls()
        self.create_heptagon_vertices()
        self.draw_balls()
        self.update()

    def create_balls(self):
        for i in range(NUM_BALLS):
            self.balls.append(
                Ball(
                    self.center_x,
                    self.center_y,
                    0,
                    0,
                    (i + 1) * 0.05,
                    COLORS[i % len(COLORS)],
                    i + 1,
                )
            )  # Initial drop from center

    def create_heptagon_vertices(self):
        for i in range(self.heptagon_sides):
            angle = math.radians(i * 360 / self.heptagon_sides)
            x = self.center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = self.center_y + HEPTAGON_RADIUS * math.sin(angle)
            self.heptagon_vertices.append((x, y))

    def draw_balls(self):
        self.canvas.delete("all")
        # Draw Heptagon
        rotated_vertices = []
        for x, y in self.heptagon_vertices:
            rotated_x, rotated_y = rotate_point(
                x, y, self.rotation_angle, self.center_x, self.center_y
            )
            rotated_vertices.append((rotated_x, rotated_y))
        self.canvas.create_polygon(rotated_vertices, fill="", outline="black")

        # Draw Balls
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS,
                ball.y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )

            # Draw number
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), font=("Arial", 10), fill="black"
            )

    def handle_wall_collision(self, ball: Ball):
        for i in range(self.heptagon_sides):
            p1 = self.heptagon_vertices[i]
            p2 = self.heptagon_vertices[(i + 1) % self.heptagon_sides]

            # Rotate wall points
            rp1 = rotate_point(
                p1[0], p1[1], self.rotation_angle, self.center_x, self.center_y
            )
            rp2 = rotate_point(
                p2[0], p2[1], self.rotation_angle, self.center_x, self.center_y
            )

            # Check if the ball is near the line segment
            dx = rp2[0] - rp1[0]
            dy = rp2[1] - rp1[1]
            if dx == 0 and dy == 0:
                continue  # Avoid zero-length line segements

            ball_to_p1_x = ball.x - rp1[0]
            ball_to_p1_y = ball.y - rp1[1]

            # Project ball onto the line segment
            t = (ball_to_p1_x * dx + ball_to_p1_y * dy) / (dx * dx + dy * dy)

            # Clamp t to the line segment
            t = max(0, min(1, t))

            # Calculate the closest point on the line segment
            closest_x = rp1[0] + t * dx
            closest_y = rp1[1] + t * dy

            # Calculate the distance between ball and closest point
            dist = distance(ball.x, ball.y, closest_x, closest_y)

            if dist < BALL_RADIUS:
                # Collision detected

                # Calculate the normal vector
                nx = (ball.x - closest_x) / dist
                ny = (ball.y - closest_y) / dist

                # Separate balls
                overlap = BALL_RADIUS - dist
                ball.x += overlap * nx
                ball.y += overlap * ny

                # Calculate the dot product of ball velocity and normal
                dot_product = ball.vx * nx + ball.vy * ny

                # Reflect the velocity
                ball.vx -= 2 * dot_product * nx * BOUNCE_REDUCTION
                ball.vy -= 2 * dot_product * ny * BOUNCE_REDUCTION

                # Add a bit of spin
                ball.spin += 0.02 * (
                    nx * ball.vy - ny * ball.vx
                )  # simplified model for spin

    def update(self):
        # Update Heptagon Rotation
        self.rotation_angle = (
            self.rotation_angle + 360 / (5 * 60)
        ) % 360  # 360 degrees / 5 seconds

        # Update Balls
        for ball in self.balls:
            ball.update()

            # Handle Wall Collisions
            self.handle_wall_collision(ball)

            # Keep the bounce height not exceeding the heptagon radius
            if ball.y + BALL_RADIUS > HEIGHT:
                ball.y = HEIGHT - BALL_RADIUS
                ball.vy *= -BOUNCE_REDUCTION  # Bounce
            if ball.y - BALL_RADIUS < 0:
                ball.y = BALL_RADIUS
                ball.vy *= -BOUNCE_REDUCTION  # Bounce
            if ball.x + BALL_RADIUS > WIDTH:
                ball.x = WIDTH - BALL_RADIUS
                ball.vx *= -BOUNCE_REDUCTION  # Bounce
            if ball.x - BALL_RADIUS < 0:
                ball.x = BALL_RADIUS
                ball.vx *= -BOUNCE_REDUCTION  # Bounce
        # Ball-Ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                resolve_ball_collision(self.balls[i], self.balls[j])

        # Draw everything
        self.draw_balls()
        self.master.after(16, self.update)  # ~60 FPS


root = tk.Tk()
game = Game(root)
root.mainloop()
