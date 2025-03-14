import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys

# Constants
WIDTH = 800
HEIGHT = 600
BALL_RADIUS = 15
HEPTAGON_RADIUS = 200
NUM_BALLS = 20
GRAVITY = 0.5
FRICTION = 0.02
SPIN_SPEED = 360 / 5  # degrees per second
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
BOUNCE_DAMPING = 0.7  # Reduce bounce height


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    rotation: float = 0.0


@dataclass
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    angle: float


def create_heptagon_vertices(
    center_x: float, center_y: float, radius: float
) -> List[Tuple[float, float]]:
    vertices = []
    for i in range(7):
        angle = 2 * math.pi * i / 7
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    return vertices


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def collision_detection(ball1: Ball, ball2: Ball) -> bool:
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    return dist <= ball1.radius + ball2.radius


def collision_response(ball1: Ball, ball2: Ball):
    # Calculate the normal vector
    nx = ball2.x - ball1.x
    ny = ball2.y - ball1.y
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
    nx /= dist
    ny /= dist

    # Calculate the relative velocity
    dvx = ball1.vx - ball2.vx
    dvy = ball1.vy - ball2.vy

    # Calculate the dot product of the relative velocity and the normal vector
    dot_product = dvx * nx + dvy * ny

    # If the balls are moving away from each other, no collision response is needed
    if dot_product > 0:
        return

    # Calculate the impulse
    impulse = (2 * dot_product) / (1 + 1)  # Assuming equal mass

    # Apply the impulse to the velocities
    ball1.vx -= impulse * nx
    ball1.vy -= impulse * ny
    ball2.vx += impulse * nx
    ball2.vy += impulse * ny


def heptagon_collision(ball: Ball, heptagon: Heptagon) -> bool:
    vertices = create_heptagon_vertices(
        heptagon.center_x, heptagon.center_y, heptagon.radius
    )
    min_dist = float("inf")
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]

        # Calculate distance from ball to line segment
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            dist = distance(ball.x, ball.y, x1, y1)
        else:
            t = ((ball.x - x1) * dx + (ball.y - y1) * dy) / (dx * dx + dy * dy)
            t = max(0, min(1, t))
            closest_x = x1 + t * dx
            closest_y = y1 + t * dy
            dist = distance(ball.x, ball.y, closest_x, closest_y)

        min_dist = min(min_dist, dist)

    return min_dist <= ball.radius


def heptagon_collision_response(ball: Ball, heptagon: Heptagon):
    vertices = create_heptagon_vertices(
        heptagon.center_x, heptagon.center_y, heptagon.radius
    )
    min_dist = float("inf")
    closest_point = None
    closest_edge = None

    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]

        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            dist = distance(ball.x, ball.y, x1, y1)
            if dist < min_dist:
                min_dist = dist
                closest_point = (x1, y1)
                closest_edge = (i, (i + 1) % len(vertices))
        else:
            t = ((ball.x - x1) * dx + (ball.y - y1) * dy) / (dx * dx + dy * dy)
            t = max(0, min(1, t))
            closest_x = x1 + t * dx
            closest_y = y1 + t * dy
            dist = distance(ball.x, ball.y, closest_x, closest_y)
            if dist < min_dist:
                min_dist = dist
                closest_point = (closest_x, closest_y)
                closest_edge = (i, (i + 1) % len(vertices))

    if closest_point:
        nx = ball.x - closest_point[0]
        ny = ball.y - closest_point[1]

        # Normalize the normal vector
        norm = math.sqrt(nx**2 + ny**2)
        if norm > 0:
            nx /= norm
            ny /= norm

        # Calculate the relative velocity
        dot_product = ball.vx * nx + ball.vy * ny

        # Apply the impulse
        impulse = -dot_product * BOUNCE_DAMPING
        ball.vx += impulse * nx
        ball.vy += impulse * ny


def update(balls: List[Ball], heptagon: Heptagon, delta_time: float):
    # Update heptagon angle
    heptagon.angle += SPIN_SPEED * delta_time * math.pi / 180

    for ball in balls:
        # Apply gravity
        ball.vy += GRAVITY * delta_time

        # Apply friction
        ball.vx *= 1 - FRICTION
        ball.vy *= 1 - FRICTION

        # Update position
        ball.x += ball.vx * delta_time
        ball.y += ball.vy * delta_time

        # Heptagon collision
        if heptagon_collision(ball, heptagon):
            heptagon_collision_response(ball, heptagon)

        # Bounce off the walls
        if ball.x - ball.radius < 0 or ball.x + ball.radius > WIDTH:
            ball.vx *= -1
        if ball.y - ball.radius < 0 or ball.y + ball.radius > HEIGHT:
            ball.vy *= -1

        # Ball-ball collision
        for other_ball in balls:
            if ball != other_ball and collision_detection(ball, other_ball):
                collision_response(ball, other_ball)

        # Update rotation
        ball.rotation += (
            (ball.vx * (ball.y - HEIGHT / 2) - ball.vy * (ball.x - WIDTH / 2))
            * delta_time
            * 0.01
        )


class BallCanvas(tk.Canvas):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, width=WIDTH, height=HEIGHT, bg="black", **kwargs)
        self.balls = []
        self.heptagon = Heptagon(WIDTH / 2, HEIGHT / 2, HEPTAGON_RADIUS, 0)
        self.init_balls()
        self.running = True
        self.animate()

    def init_balls(self):
        for i in range(NUM_BALLS):
            ball = Ball(
                x=WIDTH / 2,
                y=HEIGHT / 2,
                vx=np.random.uniform(-5, 5),
                vy=np.random.uniform(-5, 5),
                radius=BALL_RADIUS,
                color=COLORS[i % len(COLORS)],
                number=i + 1,
            )
            self.balls.append(ball)

    def animate(self):
        if self.running:
            delta_time = 0.016  # Fixed time step
            update(self.balls, self.heptagon, delta_time)
            self.draw()
            self.after(16, self.animate)  # Approximately 60 FPS

    def draw(self):
        self.delete("all")

        # Draw heptagon
        vertices = create_heptagon_vertices(
            self.heptagon.center_x, self.heptagon.center_y, self.heptagon.radius
        )
        self.create_polygon(vertices, fill="", outline="white")

        # Draw balls
        for ball in self.balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            self.create_oval(x0, y0, x1, y1, fill=ball.color, outline="white")
            self.create_text(
                ball.x, ball.y, text=str(ball.number), fill="white", font=("Arial", 10)
            )


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = BallCanvas(root)
    canvas.pack()
    root.mainloop()


if __name__ == "__main__":
    main()
