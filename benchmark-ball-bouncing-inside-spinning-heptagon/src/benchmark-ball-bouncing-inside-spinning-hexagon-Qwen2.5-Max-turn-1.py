import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 350
BALL_RADIUS = 15
GRAVITY = 0.2
FRICTION = 0.99
SPIN_SPEED = 360 / 5  # Degrees per second
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


@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    color: str
    spin: float = 0.0


@dataclass
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    angle: float = 0.0


def create_heptagon_points(center_x, center_y, radius, angle):
    points = []
    for i in range(7):
        theta = math.radians(i * 360 / 7 + angle)
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        points.append((x, y))
    return points


def point_in_polygon(px, py, polygon):
    """Check if a point is inside a polygon using ray casting."""
    n = len(polygon)
    inside = False
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > py) != (y2 > py)) and (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def resolve_ball_wall_collision(ball, polygon):
    """Resolve collision between ball and heptagon walls."""
    closest_point = None
    min_distance = float("inf")
    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]
        # Find closest point on line segment to ball
        dx, dy = x2 - x1, y2 - y1
        t = max(0, min(1, ((ball.x - x1) * dx + (ball.y - y1) * dy) / (dx**2 + dy**2)))
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        distance = math.hypot(ball.x - closest_x, ball.y - closest_y)
        if distance < min_distance:
            min_distance = distance
            closest_point = (closest_x, closest_y)

    if min_distance <= BALL_RADIUS:
        nx, ny = (ball.x - closest_point[0]), (ball.y - closest_point[1])
        length = math.hypot(nx, ny)
        if length > 0:
            nx /= length
            ny /= length
        overlap = BALL_RADIUS - min_distance
        ball.x += nx * overlap
        ball.y += ny * overlap
        dot = ball.vx * nx + ball.vy * ny
        ball.vx -= 2 * dot * nx
        ball.vy -= 2 * dot * ny
        ball.vx *= FRICTION
        ball.vy *= FRICTION


def resolve_ball_ball_collision(b1, b2):
    """Resolve collision between two balls."""
    dx, dy = b2.x - b1.x, b2.y - b1.y
    distance = math.hypot(dx, dy)
    if distance < 2 * BALL_RADIUS:
        nx, ny = dx / distance, dy / distance
        overlap = 2 * BALL_RADIUS - distance
        b1.x -= nx * overlap / 2
        b1.y -= ny * overlap / 2
        b2.x += nx * overlap / 2
        b2.y += ny * overlap / 2
        kx, ky = b1.vx - b2.vx, b1.vy - b2.vy
        p = 2 * (nx * kx + ny * ky) / 2
        b1.vx -= p * nx
        b1.vy -= p * ny
        b2.vx += p * nx
        b2.vy += p * ny
        b1.vx *= FRICTION
        b1.vy *= FRICTION
        b2.vx *= FRICTION
        b2.vy *= FRICTION


def update_physics(balls, heptagon, dt):
    """Update physics simulation."""
    polygon = create_heptagon_points(
        heptagon.center_x, heptagon.center_y, heptagon.radius, heptagon.angle
    )
    for ball in balls:
        ball.vy += GRAVITY * dt
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        ball.spin += ball.vx * dt * 0.1
        resolve_ball_wall_collision(ball, polygon)

    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            resolve_ball_ball_collision(balls[i], balls[j])


def draw_scene(canvas, balls, heptagon):
    """Draw the scene on the canvas."""
    canvas.delete("all")
    # Draw heptagon
    polygon = create_heptagon_points(
        heptagon.center_x, heptagon.center_y, heptagon.radius, heptagon.angle
    )
    canvas.create_polygon(
        *[coord for point in polygon for coord in point],
        outline="black",
        fill="",
        width=2
    )
    # Draw balls
    for ball in balls:
        x1, y1 = ball.x - BALL_RADIUS, ball.y - BALL_RADIUS
        x2, y2 = ball.x + BALL_RADIUS, ball.y + BALL_RADIUS
        canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")
        canvas.create_text(ball.x, ball.y, text=str(ball.id), fill="white")


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    # Initialize objects
    balls = [
        Ball(id=i + 1, x=WIDTH / 2, y=HEIGHT / 2, vx=0, vy=0, color=BALL_COLORS[i])
        for i in range(20)
    ]
    heptagon = Heptagon(center_x=WIDTH / 2, center_y=HEIGHT / 2, radius=HEPTAGON_RADIUS)

    def update():
        dt = 0.01
        heptagon.angle += SPIN_SPEED * dt
        update_physics(balls, heptagon, dt)
        draw_scene(canvas, balls, heptagon)
        root.after(int(dt * 1000), update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
