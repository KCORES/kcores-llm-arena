import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
BALL_COLORS = [
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
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (60 FPS)


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    angle: float
    spin: float
    number: int
    color: str


@dataclass
class Heptagon:
    radius: float
    angle: float


def rotate_point(x: float, y: float, angle: float) -> Tuple[float, float]:
    """Rotate a point around the origin by a given angle."""
    rad = math.radians(angle)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    return x * cos_a - y * sin_a, x * sin_a + y * cos_a


def get_heptagon_points(heptagon: Heptagon) -> List[Tuple[float, float]]:
    """Get the vertices of the heptagon."""
    points = []
    for i in range(HEPTAGON_SIDES):
        angle = 360 / HEPTAGON_SIDES * i + heptagon.angle
        x = heptagon.radius * math.cos(math.radians(angle))
        y = heptagon.radius * math.sin(math.radians(angle))
        points.append((x, y))
    return points


def check_collision(ball1: Ball, ball2: Ball) -> bool:
    """Check if two balls are colliding."""
    dx = ball1.x - ball2.x
    dy = ball1.y - ball2.y
    distance = math.hypot(dx, dy)
    return distance <= 2 * BALL_RADIUS


def resolve_collision(ball1: Ball, ball2: Ball):
    """Resolve the collision between two balls."""
    dx = ball1.x - ball2.x
    dy = ball1.y - ball2.y
    distance = math.hypot(dx, dy)
    if distance == 0:
        return

    # Normalize the collision vector
    nx, ny = dx / distance, dy / distance

    # Relative velocity
    rvx = ball1.vx - ball2.vx
    rvy = ball1.vy - ball2.vy

    # Velocity along the normal
    vel_along_normal = rvx * nx + rvy * ny

    # Do not resolve if balls are moving apart
    if vel_along_normal > 0:
        return

    # Perfectly elastic collision
    e = 1.0
    j = -(1 + e) * vel_along_normal
    j /= 1 / 1 + 1 / 1  # Assuming equal mass

    # Apply impulse
    ball1.vx += j * nx
    ball1.vy += j * ny
    ball2.vx -= j * nx
    ball2.vy -= j * ny


def update_balls(balls: List[Ball], heptagon: Heptagon):
    """Update the positions and velocities of the balls."""
    for ball in balls:
        # Apply gravity
        ball.vy += GRAVITY

        # Update position
        ball.x += ball.vx
        ball.y += ball.vy

        # Apply friction
        ball.vx *= FRICTION
        ball.vy *= FRICTION

        # Update spin
        ball.angle += ball.spin
        ball.spin *= FRICTION

        # Check collision with heptagon walls
        points = get_heptagon_points(heptagon)
        for i in range(HEPTAGON_SIDES):
            x1, y1 = points[i]
            x2, y2 = points[(i + 1) % HEPTAGON_SIDES]

            # Vector from ball to wall segment
            dx = x2 - x1
            dy = y2 - y1
            segment_length = math.hypot(dx, dy)
            nx, ny = -dy / segment_length, dx / segment_length  # Normal vector

            # Distance from ball to wall
            dist = (ball.x - x1) * nx + (ball.y - y1) * ny

            if abs(dist) <= BALL_RADIUS:
                # Reflect velocity
                dot = ball.vx * nx + ball.vy * ny
                ball.vx -= 2 * dot * nx
                ball.vy -= 2 * dot * ny

                # Move ball outside the wall
                ball.x += (BALL_RADIUS - dist) * nx
                ball.y += (BALL_RADIUS - dist) * ny

        # Check collisions between balls
        for other in balls:
            if other != ball and check_collision(ball, other):
                resolve_collision(ball, other)


def draw_heptagon(canvas: tk.Canvas, heptagon: Heptagon):
    """Draw the heptagon on the canvas."""
    points = get_heptagon_points(heptagon)
    canvas.create_polygon(
        *[coord for point in points for coord in point],
        outline="black",
        fill="",
        width=2
    )


def draw_balls(canvas: tk.Canvas, balls: List[Ball]):
    """Draw the balls on the canvas."""
    for ball in balls:
        x1, y1 = ball.x - BALL_RADIUS, ball.y - BALL_RADIUS
        x2, y2 = ball.x + BALL_RADIUS, ball.y + BALL_RADIUS
        canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")
        canvas.create_text(ball.x, ball.y, text=str(ball.number), fill="white")


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=800, height=800)
    canvas.pack()

    # Initialize heptagon
    heptagon = Heptagon(radius=HEPTAGON_RADIUS, angle=0)

    # Initialize balls
    balls = []
    for i in range(20):
        angle = np.random.uniform(0, 360)
        speed = np.random.uniform(2, 5)
        vx = speed * math.cos(math.radians(angle))
        vy = speed * math.sin(math.radians(angle))
        balls.append(
            Ball(
                x=0,
                y=0,
                vx=vx,
                vy=vy,
                angle=0,
                spin=np.random.uniform(-1, 1),
                number=i + 1,
                color=BALL_COLORS[i],
            )
        )

    def update():
        nonlocal heptagon
        canvas.delete("all")

        # Update heptagon angle
        heptagon.angle += SPIN_SPEED
        if heptagon.angle >= 360:
            heptagon.angle -= 360

        # Update balls
        update_balls(balls, heptagon)

        # Draw heptagon and balls
        draw_heptagon(canvas, heptagon)
        draw_balls(canvas, balls)

        root.after(16, update)  # ~60 FPS

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
