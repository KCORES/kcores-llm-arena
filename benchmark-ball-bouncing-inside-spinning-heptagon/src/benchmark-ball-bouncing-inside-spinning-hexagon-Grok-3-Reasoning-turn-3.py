import tkinter as tk
from math import sin, cos, pi
import numpy as np
from dataclasses import dataclass
import random

# Constants
WIDTH = 800
HEIGHT = 800
CX = WIDTH / 2
CY = HEIGHT / 2
R = 300  # Heptagon radius
N_BALLS = 20
BALL_RADIUS = 10
G = 600  # Gravity in pixels/s^2
DT = 0.01  # Time step in seconds
OMEGA = 2 * pi / 5  # Angular velocity of heptagon (360° in 5s)
E = 0.8  # Coefficient of restitution
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


# Ball class definition
@dataclass
class Ball:
    position: np.ndarray  # [x, y]
    velocity: np.ndarray  # [vx, vy]
    omega: float  # Angular velocity (rad/s)
    theta: float  # Angular position (rad)
    color: str
    number: int
    oval_id: int = None  # Tkinter oval object ID
    text_id: int = None  # Tkinter text object ID


# Compute initial heptagon vertices around (0,0)
def compute_initial_vertices():
    vertices = []
    for k in range(7):
        theta = 2 * pi * k / 7
        x = R * cos(theta)
        y = R * sin(theta)
        vertices.append(np.array([x, y]))
    return vertices


# Compute initial inward normals for each wall
def compute_initial_normals(vertices):
    normals = []
    for i in range(7):
        A = vertices[i]
        B = vertices[(i + 1) % 7]
        direction = B - A
        n = np.array([-direction[1], direction[0]])
        n = n / np.linalg.norm(n)
        M = (A + B) / 2
        # Ensure n points inward (towards center at (0,0))
        if np.dot(M, n) > 0:
            n = -n
        normals.append(n)
    return normals


# Find closest point on a line segment to a point
def closest_point_on_segment(A, B, C):
    direction = B - A
    dir_len_sq = np.dot(direction, direction)
    if dir_len_sq == 0:
        return A
    t = np.dot(C - A, direction) / dir_len_sq
    t = max(0, min(1, t))
    return A + t * direction


# Simulation update function
def update(t):
    phi = OMEGA * t
    c, s = cos(phi), sin(phi)
    rotation = np.array([[c, -s], [s, c]])

    # Update heptagon position
    current_vertices = [rotation @ v + [CX, CY] for v in initial_vertices]
    current_normals = [rotation @ n for n in initial_normals]
    for i, line_id in enumerate(wall_ids):
        A = current_vertices[i]
        B = current_vertices[(i + 1) % 7]
        canvas.coords(line_id, A[0], A[1], B[0], B[1])

    # Update balls
    for ball in balls:
        # Apply gravity and update position
        ball.velocity[1] += G * DT
        ball.position += ball.velocity * DT

        # Ball-wall collisions
        for i in range(7):
            A = current_vertices[i]
            B = current_vertices[(i + 1) % 7]
            C = ball.position
            P = closest_point_on_segment(A, B, C)
            distance = np.linalg.norm(P - C)
            if distance < BALL_RADIUS:
                n = current_normals[i]
                # Wall velocity at contact point
                v_w = OMEGA * np.array([-(P[1] - CY), P[0] - CX])
                v_r = ball.velocity - v_w
                v_r_n = np.dot(v_r, n)
                if v_r_n < 0:  # Ball moving towards wall
                    # Normal collision response
                    ball.velocity -= (1 + E) * v_r_n * n
                    # Friction and rotation
                    t = np.array([-n[1], n[0]])
                    v_b_t = np.dot(ball.velocity, t)
                    v_w_t = np.dot(v_w, t)
                    ball.omega = (v_b_t - v_w_t) / BALL_RADIUS

        # Ball-ball collisions
        for other in balls:
            if other is ball:
                continue
            delta = other.position - ball.position
            dist = np.linalg.norm(delta)
            if dist < 2 * BALL_RADIUS and dist > 0:
                n = delta / dist
                v_rel = ball.velocity - other.velocity
                v_rel_n = np.dot(v_rel, n)
                if v_rel_n > 0:  # Balls approaching
                    j = (1 + E) * v_rel_n / 2  # Equal masses
                    ball.velocity -= j * n
                    other.velocity += j * n

        # Update rotation
        ball.theta += ball.omega * DT

        # Update graphics
        x, y = ball.position
        canvas.coords(
            ball.oval_id,
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
        )
        canvas.coords(ball.text_id, x, y)
        theta_deg = ball.theta * 180 / pi
        canvas.itemconfig(ball.text_id, angle=theta_deg)

    # Schedule next update
    root.after(int(DT * 1000), update, t + DT)


# Main setup
if __name__ == "__main__":
    # Initialize window
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    # Heptagon setup
    initial_vertices = compute_initial_vertices()
    initial_normals = compute_initial_normals(initial_vertices)
    wall_ids = [canvas.create_line(0, 0, 0, 0, width=2) for _ in range(7)]

    # Balls setup
    balls = [
        Ball(
            position=np.array(
                [CX + random.uniform(-0.1, 0.1), CY + random.uniform(-0.1, 0.1)]
            ),
            velocity=np.array([0.0, 0.0]),
            omega=0.0,
            theta=0.0,
            color=COLORS[i],
            number=i + 1,
        )
        for i in range(N_BALLS)
    ]
    for ball in balls:
        x, y = ball.position
        ball.oval_id = canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=ball.color,
        )
        ball.text_id = canvas.create_text(
            x, y, text=str(ball.number), font=("Arial", 8), anchor="center", angle=0
        )

    # Start simulation
    t = 0
    update(t)
    root.mainloop()
