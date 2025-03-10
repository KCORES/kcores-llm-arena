import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
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
NUM_BALLS = 20


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    color: str
    number: int
    spin_angle: float = 0.0


def rotate_point(cx, cy, x, y, angle):
    """Rotate a point (x, y) around (cx, cy) by angle (in degrees)."""
    rad = math.radians(angle)
    s, c = math.sin(rad), math.cos(rad)
    dx, dy = x - cx, y - cy
    return cx + dx * c - dy * s, cy + dx * s + dy * c


def heptagon_vertices(radius, center_x, center_y, rotation_angle):
    """Calculate the vertices of a heptagon."""
    angles = [rotation_angle + i * 360 / 7 for i in range(7)]
    return [
        rotate_point(center_x, center_y, center_x + radius, center_y, a) for a in angles
    ]


def point_in_polygon(px, py, polygon):
    """Check if a point (px, py) is inside a polygon."""
    n = len(polygon)
    inside = False
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > py) != (y2 > py)) and (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def resolve_ball_wall_collision(ball, polygon):
    """Resolve collision between a ball and the heptagon walls."""
    min_overlap = float("inf")
    normal = None
    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]
        edge_vx, edge_vy = x2 - x1, y2 - y1
        length_sq = edge_vx**2 + edge_vy**2
        t = max(
            0, min(1, ((ball.x - x1) * edge_vx + (ball.y - y1) * edge_vy) / length_sq)
        )
        closest_x, closest_y = x1 + t * edge_vx, y1 + t * edge_vy
        dist_sq = (ball.x - closest_x) ** 2 + (ball.y - closest_y) ** 2
        if dist_sq < BALL_RADIUS**2:
            overlap = BALL_RADIUS - math.sqrt(dist_sq)
            if overlap < min_overlap:
                min_overlap = overlap
                nx, ny = ball.x - closest_x, ball.y - closest_y
                norm = math.hypot(nx, ny)
                normal = (nx / norm, ny / norm)
    if normal:
        ball.vx -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[0]
        ball.vy -= 2 * (ball.vx * normal[0] + ball.vy * normal[1]) * normal[1]
        ball.x += normal[0] * min_overlap
        ball.y += normal[1] * min_overlap


def resolve_ball_ball_collision(b1, b2):
    """Resolve collision between two balls."""
    dx, dy = b2.x - b1.x, b2.y - b1.y
    dist_sq = dx**2 + dy**2
    if dist_sq < (2 * BALL_RADIUS) ** 2:
        dist = math.sqrt(dist_sq)
        nx, ny = dx / dist, dy / dist
        rel_vx, rel_vy = b2.vx - b1.vx, b2.vy - b1.vy
        vel_along_normal = rel_vx * nx + rel_vy * ny
        if vel_along_normal > 0:
            e = 0.8  # Coefficient of restitution
            j = -(1 + e) * vel_along_normal / (1 / 1 + 1 / 1)
            impulse_x, impulse_y = j * nx, j * ny
            b1.vx -= impulse_x / 1
            b1.vy -= impulse_y / 1
            b2.vx += impulse_x / 1
            b2.vy += impulse_y / 1
        overlap = 2 * BALL_RADIUS - dist
        b1.x -= overlap * nx / 2
        b1.y -= overlap * ny / 2
        b2.x += overlap * nx / 2
        b2.y += overlap * ny / 2


def update_physics(balls, polygon, dt):
    """Update the physics simulation."""
    for ball in balls:
        ball.vy += GRAVITY * dt
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        resolve_ball_wall_collision(ball, polygon)
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            resolve_ball_ball_collision(balls[i], balls[j])


def draw_scene(canvas, balls, polygon, rotation_angle):
    """Draw the scene on the canvas."""
    canvas.delete("all")
    # Draw heptagon
    polygon_coords = [coord for vertex in polygon for coord in vertex]
    canvas.create_polygon(polygon_coords, outline="black", fill="", width=2)
    # Draw balls
    for ball in balls:
        x, y = rotate_point(WIDTH / 2, HEIGHT / 2, ball.x, ball.y, rotation_angle)
        canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=ball.color,
            outline="black",
        )
        canvas.create_text(
            x, y, text=str(ball.number), fill="white", font=("Arial", 10)
        )


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    # Initialize balls
    balls = [
        Ball(WIDTH / 2, HEIGHT / 2, 0, 0, BALL_COLORS[i], i + 1)
        for i in range(NUM_BALLS)
    ]

    # Animation loop
    def animate(time):
        dt = 0.016  # Approximate frame time
        rotation_angle = (time * SPIN_SPEED) % 360
        polygon = heptagon_vertices(
            HEPTAGON_RADIUS, WIDTH / 2, HEIGHT / 2, rotation_angle
        )
        update_physics(balls, polygon, dt)
        draw_scene(canvas, balls, polygon, rotation_angle)
        root.after(16, animate, time + dt)

    animate(0)
    root.mainloop()


if __name__ == "__main__":
    main()
