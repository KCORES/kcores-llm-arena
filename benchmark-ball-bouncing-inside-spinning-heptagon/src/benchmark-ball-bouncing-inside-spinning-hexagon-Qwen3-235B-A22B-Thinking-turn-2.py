import tkinter as tk
import math
import random
from dataclasses import dataclass


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


# Constants
BALL_RADIUS = 10
WINDOW_SIZE = 800
HEPTAGON_CENTER = (WINDOW_SIZE // 2, WINDOW_SIZE // 2)
HEPTAGON_RADIUS = 350
GRAVITY = (0, 0.5)
FRICTION = 0.99
ANGULAR_FRICTION = 0.995
RESTITUTION = 0.8
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
NUM_BALLS = 20

balls = []


# Helper Functions
def add_vec(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub_vec(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul_vec(v, s):
    return (v[0] * s, v[1] * s)


def dot_vec(a, b):
    return a[0] * b[0] + a[1] * b[1]


def length_vec(v):
    return math.hypot(v[0], v[1])


def normalize_vec(v):
    l = length_vec(v)
    return (v[0] / l, v[1] / l) if l != 0 else (0, 0)


# Heptagon Geometry
def compute_vertices(angle):
    cx, cy = HEPTAGON_CENTER
    vertices = []
    for i in range(7):
        theta = angle + 2 * math.pi * i / 7
        x = cx + HEPTAGON_RADIUS * math.cos(theta)
        y = cy + HEPTAGON_RADIUS * math.sin(theta)
        vertices.append((x, y))
    return vertices


# Wall Collision Detection
def closest_point_on_segment(P, A, B):
    AB = sub_vec(B, A)
    AP = sub_vec(P, A)
    ab2 = dot_vec(AB, AB)
    ap_ab = dot_vec(AP, AB)
    t = max(0.0, min(1.0, ap_ab / ab2 if ab2 != 0 else 0.0))
    return add_vec(A, mul_vec(AB, t))


def detect_wall_collision(ball, A, B):
    P = (ball.x, ball.y)
    Q = closest_point_on_segment(P, A, B)
    dist = math.hypot(P[0] - Q[0], P[1] - Q[1])
    if dist < ball.radius:
        edge_dir = sub_vec(B, A)
        normal = (edge_dir[1], -edge_dir[0])
        normal = normalize_vec(normal)
        center_to_Q = sub_vec(P, Q)
        if dot_vec(center_to_Q, normal) < 0:
            normal = (-normal[0], -normal[1])
        return True, normal, Q
    return False, None, None


# Wall Collision Response
def resolve_wall_collision(ball, normal, contact, wall_vel):
    v_ball = (ball.vx, ball.vy)
    v_rel = sub_vec(v_ball, wall_vel)
    v_dot_n = dot_vec(v_rel, normal)
    if v_dot_n > 0:
        return

    j = -(1 + RESTITUTION) * v_dot_n
    impulse = mul_vec(normal, j)
    ball.vx += impulse[0]
    ball.vy += impulse[1]

    tangent = (-normal[1], normal[0])
    v_tangent = dot_vec(v_rel, tangent)
    ball.angular_velocity -= 0.1 * v_tangent / ball.radius

    distCP = sub_vec((ball.x, ball.y), contact)
    dist = length_vec(distCP)
    if dist > 0:
        correction = mul_vec(normalize_vec(distCP), ball.radius - dist)
        ball.x += correction[0]
        ball.y += correction[1]


# Ball Collision Detection & Resolution
def detect_ball_collision(a, b):
    dx = b.x - a.x
    dy = b.y - a.y
    distance = math.hypot(dx, dy)
    return (True, distance) if distance < a.radius + b.radius else (False, distance)


def resolve_ball_collision(a, b, distance):
    if distance == 0:
        return
    nx = (b.x - a.x) / distance
    ny = (b.y - a.y) / distance

    dvx = b.vx - a.vx
    dvy = b.vy - a.vy
    vn = dvx * nx + dvy * ny

    if vn > 0:
        return

    p = -(1 + RESTITUTION) * vn / 2
    a.vx -= p * nx
    a.vy -= p * ny
    b.vx += p * nx
    b.vy += p * ny

    overlap = a.radius + b.radius - distance
    if overlap > 0:
        corr_x = nx * overlap / 2
        corr_y = ny * overlap / 2
        a.x -= corr_x
        a.y -= corr_y
        b.x += corr_x
        b.y += corr_y

    tangent = (-ny, nx)
    tv = dvx * tangent[0] + dvy * tangent[1]
    a.angular_velocity += 0.05 * tv / a.radius
    b.angular_velocity += 0.05 * tv / b.radius


# Physics Update
def update_balls(delta_time):
    cx, cy = HEPTAGON_CENTER
    angle = last_theta + (2 * math.pi / 5) * delta_time
    vertices = compute_vertices(angle)

    # Physics Step
    for ball in balls:
        ball.vy += GRAVITY[1] * delta_time * 10
        ball.vx *= 1 - FRICTION * delta_time * 10
        ball.vy *= 1 - FRICTION * delta_time * 10
        ball.angle += ball.angular_velocity * delta_time * 10
        ball.angular_velocity *= ANGULAR_FRICTION
        ball.x += ball.vx * delta_time * 10
        ball.y += ball.vy * delta_time * 10

    # Wall Collisions
    for i in range(7):
        A, B = vertices[i], vertices[(i + 1) % 7]
        for ball in balls:
            rel_x = contact[0] - cx
            rel_y = contact[1] - cy
            wx = -omega * rel_y
            wy = omega * rel_x
            collided, normal, contact = detect_wall_collision(ball, A, B)
            if collided:
                resolve_wall_collision(ball, normal, contact, (wx, wy))

    # Ball-Ball Collisions
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            a, b = balls[i], balls[j]
            collided, d = detect_ball_collision(a, b)
            if collided:
                resolve_ball_collision(a, b, d)


# Drawing
def draw():
    canvas.delete("all")
    vertices = compute_vertices(last_theta)
    coords = []
    for pt in vertices:
        coords.extend(pt)
    canvas.create_polygon(*coords, fill="", outline="black", width=2)

    for ball in balls:
        x0 = ball.x - ball.radius
        y0 = ball.y - ball.radius
        x1 = ball.x + ball.radius
        y1 = ball.y + ball.radius
        canvas.create_oval(x0, y0, x1, y1, fill=ball.color)
        dir_x = math.cos(ball.angle) * ball.radius * 0.7
        dir_y = math.sin(ball.angle) * ball.radius * 0.7
        canvas.create_line(
            ball.x, ball.y, ball.x + dir_x, ball.y + dir_y, fill="white", width=2
        )
        canvas.create_text(ball.x, ball.y, text=str(ball.number), fill="black")


# Main Loop
last_time = 0
last_theta = 0
omega = 2 * math.pi / 5  # rad/s


def main_loop():
    global last_time, last_theta
    current_time = main_loop.root._lastcall = getattr(
        main_loop.root, "_lastcall", time.time()
    )
    now = time.time()
    delta = now - current_time
    main_loop.root._lastcall = now

    if last_time != 0:
        update_balls(delta)
        last_theta += omega * delta

    last_time = now
    draw()
    root.after(16, main_loop)


# Initialize
import time

root = tk.Tk()
canvas = tk.Canvas(root, width=WINDOW_SIZE, height=WINDOW_SIZE, bg="white")
canvas.pack()

# Seed balls
for i in range(NUM_BALLS):
    ball = Ball(
        x=HEPTAGON_CENTER[0],
        y=HEPTAGON_CENTER[1],
        vx=(random.random() - 0.5) * 4,
        vy=(random.random() - 0.5) * 4,
        radius=BALL_RADIUS,
        color=COLORS[i % len(COLORS)],
        number=i + 1,
    )
    balls.append(ball)

main_loop.root = root
main_loop()
root.mainloop()
