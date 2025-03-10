import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    av: float  # Angular velocity (radians per second)
    radius: float
    color: str
    number: int


WIDTH = 800
HEIGHT = 800
CENTER_X = WIDTH / 2
CENTER_Y = HEIGHT / 2
HEPTAGON_RADIUS = 400  # Radius from center to vertices
BALL_RADIUS = 20
GRAVITY = 980  # Pixels per second squared (increased for visibility)
FRICTION = 0.99  # Linear velocity damping per frame
ANGULAR_FRICTION = 0.99  # Angular velocity damping per frame
RESTITUTION = 0.8  # Coefficient of restitution for collisions
ANGULAR_IMPACT_FACTOR = 0.05  # Factor for angular velocity change on collision

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

balls = []
for i in range(20):
    balls.append(
        Ball(
            x=CENTER_X,
            y=CENTER_Y,
            vx=0.0,
            vy=0.0,
            av=0.0,
            radius=BALL_RADIUS,
            color=COLORS[i],
            number=i + 1,
        )
    )

root = tk.Tk()
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
canvas.pack()


def get_heptagon_vertices(angle):
    vertices = []
    for i in range(7):
        theta = angle + 2 * math.pi * i / 7
        x = CENTER_X + HEPTAGON_RADIUS * math.cos(theta)
        y = CENTER_Y + HEPTAGON_RADIUS * math.sin(theta)
        vertices.append((x, y))
    return vertices


def distance_point_to_segment(p, a, b):
    a = np.array(a, dtype=np.float64)
    b = np.array(b, dtype=np.float64)
    p = np.array(p, dtype=np.float64)
    ab = b - a
    ap = p - a
    if np.linalg.norm(ab) == 0:
        return np.linalg.norm(p - a)
    proj = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(proj, 0, 1)
    closest = a + t * ab
    return np.linalg.norm(p - closest)


def check_wall_collision(ball, vertices, omega, dt):
    for i in range(7):
        a, b = vertices[i], vertices[(i + 1) % 7]
        if distance_point_to_segment((ball.x, ball.y), a, b) <= ball.radius:
            resolve_wall_collision(ball, a, b, omega, dt)
            return True
    return False


def resolve_wall_collision(ball, a, b, omega, dt):
    a = np.array(a)
    b = np.array(b)
    ball_pos = np.array([ball.x, ball.y])
    ab = b - a
    ap = ball_pos - a
    proj = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(proj, 0, 1)
    closest = a + t * ab
    normal = np.array([-ab[1], ab[0]])
    normal /= np.linalg.norm(normal)

    dx_edge = closest[0] - CENTER_X
    dy_edge = closest[1] - CENTER_Y
    edge_v = np.array([omega * dy_edge, -omega * dx_edge])

    rel_v = np.array([ball.vx, ball.vy]) - edge_v
    vel_dot_normal = np.dot(rel_v, normal)
    if vel_dot_normal >= 0:
        return

    vel_tangent = rel_v - vel_dot_normal * normal
    new_rel_v = vel_tangent - RESTITUTION * vel_dot_normal * normal
    new_v = new_rel_v + edge_v

    ball.vx, ball.vy = new_v[0], new_v[1]

    tangential_speed = np.linalg.norm(vel_tangent)
    if tangential_speed > 0:
        direction = np.sign(np.cross(vel_tangent, normal))
        ball.av += direction * tangential_speed / ball.radius * ANGULAR_IMPACT_FACTOR

    overlap = ball.radius - np.linalg.norm(ball_pos - closest)
    if overlap > 0:
        move_dir = (ball_pos - closest) / np.linalg.norm(ball_pos - closest)
        ball.x += move_dir[0] * overlap
        ball.y += move_dir[1] * overlap


def check_ball_collisions(balls, dt):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            b1, b2 = balls[i], balls[j]
            dx = b2.x - b1.x
            dy = b2.y - b1.y
            dist = math.hypot(dx, dy)
            if dist < b1.radius + b2.radius:
                resolve_ball_collision(b1, b2, dx, dy, dist)


def resolve_ball_collision(b1, b2, dx, dy, dist):
    normal = np.array([dx, dy]) / dist
    rel_v = np.array([b2.vx - b1.vx, b2.vy - b1.vy])
    vel_along_normal = np.dot(rel_v, normal)
    if vel_along_normal > 0:
        return

    e = RESTITUTION
    j = -(1 + e) * vel_along_normal
    j /= 1.0 + 1.0

    impulse = j * normal
    b1.vx -= impulse[0]
    b1.vy -= impulse[1]
    b2.vx += impulse[0]
    b2.vy += impulse[1]

    vel_tangent = rel_v - vel_along_normal * normal
    tangential_speed = np.linalg.norm(vel_tangent)
    if tangential_speed > 0:
        direction = np.sign(np.cross(vel_tangent, normal))
        b1.av += direction * tangential_speed / b1.radius * ANGULAR_IMPACT_FACTOR
        b2.av -= direction * tangential_speed / b2.radius * ANGULAR_IMPACT_FACTOR

    overlap = (b1.radius + b2.radius) - dist
    if overlap > 0:
        move = normal * overlap * 0.5
        b1.x -= move[0]
        b1.y -= move[1]
        b2.x += move[0]
        b2.y += move[1]


def update(dt, angle, omega):
    angle += omega * dt
    for ball in balls:
        ball.vy += GRAVITY * dt
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        ball.av *= ANGULAR_FRICTION

    vertices = get_heptagon_vertices(angle)
    for ball in balls:
        check_wall_collision(ball, vertices, omega, dt)

    check_ball_collisions(balls, dt)

    return angle


def draw(angle):
    canvas.delete("all")
    vertices = get_heptagon_vertices(angle)
    coords = []
    for v in vertices:
        coords.extend(v)
    canvas.create_polygon(coords, outline="black", fill="", width=2)

    for ball in balls:
        angle_deg = math.degrees(ball.av)
        canvas.create_oval(
            ball.x - ball.radius,
            ball.y - ball.radius,
            ball.x + ball.radius,
            ball.y + ball.radius,
            fill=ball.color,
            outline="black",
        )
        canvas.create_text(
            ball.x,
            ball.y,
            text=str(ball.number),
            angle=angle_deg,
            font=("Arial", int(ball.radius * 1.2), "bold"),
            fill="black",
        )


def main_loop(prev_time, angle, omega):
    current_time = time.time()
    dt = current_time - prev_time
    if dt == 0:
        dt = 0.01

    angle = update(dt, angle, omega)
    draw(angle)

    root.after(16, main_loop, current_time, angle, omega)


import time

angle = 0.0
omega = 2 * math.pi / 5  # 360 degrees per 5 seconds in radians

root.after(16, main_loop, time.time(), angle, omega)
root.mainloop()
