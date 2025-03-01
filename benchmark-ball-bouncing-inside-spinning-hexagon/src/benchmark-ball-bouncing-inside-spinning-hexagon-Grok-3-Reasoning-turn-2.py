import tkinter as tk
from math import sin, cos, pi
import numpy as np
from dataclasses import dataclass
from typing import List
import sys

# --- Constants ---
WIDTH = 800  # Canvas width in pixels
HEIGHT = 800  # Canvas height in pixels
CENTER_X = WIDTH / 2  # Center x-coordinate
CENTER_Y = HEIGHT / 2  # Center y-coordinate
R = 300  # Heptagon radius (distance from center to vertex)
R_BALL = 10  # Ball radius
N_BALLS = 20  # Number of balls
G = 500  # Gravity in pixels/s^2
OMEGA = 2 * pi / 5  # Angular velocity (360° per 5 seconds = 2π/5 rad/s)
DT = 0.01  # Time step in seconds
E = 0.9  # Coefficient of restitution (bounce energy retention)
MU = 0.5  # Friction coefficient

# Colors for the 20 balls as specified
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

# --- Heptagon Definition ---
# Vertices in the rotating frame (heptagon is stationary here)
HEPTAGON_VERTICES = [
    (R * cos(2 * pi * k / 7), R * sin(2 * pi * k / 7)) for k in range(7)
]

# Walls with inward normals
HEPTAGON_WALLS = []
for k in range(7):
    v1 = HEPTAGON_VERTICES[k]
    v2 = HEPTAGON_VERTICES[(k + 1) % 7]
    dx = v2[0] - v1[0]
    dy = v2[1] - v1[1]
    length = (dx**2 + dy**2) ** 0.5
    # Outward normal rotated 90° clockwise
    outward_normal = (-dy / length, dx / length)
    # Inward normal is the negative of outward normal
    inward_normal = (dy / length, -dx / length)
    HEPTAGON_WALLS.append({"v1": v1, "v2": v2, "inward_normal": inward_normal})


# --- Ball Class ---
@dataclass
class Ball:
    r_rot: np.ndarray  # Position in rotating frame
    v_rot: np.ndarray  # Velocity in rotating frame
    phi_rot: float  # Orientation angle in rotating frame
    omega_ball: float  # Angular velocity in rotating frame
    color: str  # Ball color
    number: int  # Ball number (1 to 20)


# --- Initialize Balls ---
balls = [
    Ball(
        r_rot=np.array([0.0, 0.0]),  # Start at center
        v_rot=np.random.randn(2) * 10,  # Small random initial velocity
        phi_rot=0.0,  # Initial orientation
        omega_ball=0.0,  # Initial angular velocity
        color=COLORS[i],
        number=i + 1,
    )
    for i in range(N_BALLS)
]

# --- Tkinter Setup ---
root = tk.Tk()
root.title("Bouncing Balls in Spinning Heptagon")
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
canvas.pack()

# Heptagon lines
heptagon_lines = [
    canvas.create_line(0, 0, 0, 0, fill="black", width=2) for _ in range(7)
]

# Ball items (oval for ball, line for orientation)
ball_items = []
for ball in balls:
    oval = canvas.create_oval(0, 0, 0, 0, fill=ball.color, outline="")
    line = canvas.create_line(0, 0, 0, 0, fill="black", width=2)
    ball_items.append((oval, line))


# --- Helper Functions ---
def distance_to_segment(p, v1, v2):
    """Compute the distance from point p to line segment v1-v2."""
    line_vec = np.array(v2) - np.array(v1)
    p_vec = p - np.array(v1)
    line_len = np.linalg.norm(line_vec)
    if line_len == 0:
        return np.linalg.norm(p_vec)
    line_unit = line_vec / line_len
    proj = np.dot(p_vec, line_unit)
    if proj < 0:
        return np.linalg.norm(p - np.array(v1))
    elif proj > line_len:
        return np.linalg.norm(p - np.array(v2))
    closest = np.array(v1) + proj * line_unit
    return np.linalg.norm(p - closest)


def handle_wall_collision(ball, wall):
    """Handle collision between a ball and a heptagon wall with friction."""
    d = distance_to_segment(ball.r_rot, wall["v1"], wall["v2"])
    if d >= R_BALL:
        return
    n = np.array(wall["inward_normal"])  # Inward normal
    t = np.array([-n[1], n[0]])  # Tangent (90° counterclockwise)
    v_rel = ball.v_rot  # Relative velocity (wall is fixed)
    v_n = np.dot(v_rel, n)
    if v_n >= 0:  # Not moving towards the wall
        return
    v_t = np.dot(v_rel, t)
    v_slip = v_t - ball.omega_ball * R_BALL  # Tangential velocity at contact
    m = 1  # Mass of ball
    I = (2 / 5) * m * R_BALL**2  # Moment of inertia
    J_n = -(1 + E) * v_n * m  # Normal impulse
    J_t_stick = -v_slip / (1 / m + R_BALL**2 / I)  # Tangential impulse for sticking
    if abs(J_t_stick) <= MU * abs(J_n):
        J_t = J_t_stick  # Sticking
    else:
        J_t = -np.sign(v_slip) * MU * abs(J_n)  # Sliding
    J = J_n * n + J_t * t  # Total impulse
    ball.v_rot += J / m
    ball.omega_ball += -(R_BALL * J_t) / I


def update_heptagon(t):
    """Update the heptagon's position in the lab frame."""
    theta = OMEGA * t
    c, s = cos(theta), sin(theta)
    for k, line in enumerate(heptagon_lines):
        v1 = HEPTAGON_VERTICES[k]
        v2 = HEPTAGON_VERTICES[(k + 1) % 7]
        x1_lab = CENTER_X + c * v1[0] - s * v1[1]
        y1_lab = CENTER_Y + s * v1[0] + c * v1[1]
        x2_lab = CENTER_X + c * v2[0] - s * v2[1]
        y2_lab = CENTER_Y + s * v2[0] + c * v2[1]
        canvas.coords(line, x1_lab, y1_lab, x2_lab, y2_lab)


def update_balls(t):
    """Update the balls' positions and orientations in the lab frame."""
    theta = OMEGA * t
    c, s = cos(theta), sin(theta)
    for ball, (oval, line) in zip(balls, ball_items):
        x_rot, y_rot = ball.r_rot
        x_lab = c * x_rot - s * y_rot
        y_lab = s * x_rot + c * y_rot
        x_screen = CENTER_X + x_lab
        y_screen = CENTER_Y + y_lab
        # Update ball oval
        canvas.coords(
            oval,
            x_screen - R_BALL,
            y_screen - R_BALL,
            x_screen + R_BALL,
            y_screen + R_BALL,
        )
        # Update orientation line
        phi_lab = ball.phi_rot + theta
        dx = R_BALL * cos(phi_lab)
        dy = R_BALL * sin(phi_lab)
        canvas.coords(line, x_screen, y_screen, x_screen + dx, y_screen + dy)


# --- Simulation Loop ---
t = 0.0


def update():
    global t
    t += DT
    theta = OMEGA * t
    # Gravity in rotating frame
    g_rot = np.array([G * sin(theta), -G * cos(theta)])

    # Update physics for each ball
    for ball in balls:
        # Effective acceleration: gravity + centrifugal + Coriolis
        centrifugal = -(OMEGA**2) * ball.r_rot
        coriolis = -2 * OMEGA * np.array([-ball.v_rot[1], ball.v_rot[0]])
        a_eff = g_rot + centrifugal + coriolis
        ball.v_rot += a_eff * DT
        ball.r_rot += ball.v_rot * DT
        ball.phi_rot += ball.omega_ball * DT

    # Handle wall collisions
    for ball in balls:
        for wall in HEPTAGON_WALLS:
            handle_wall_collision(ball, wall)

    # Handle ball-ball collisions (elastic, no friction)
    for i in range(N_BALLS):
        for j in range(i + 1, N_BALLS):
            ball1 = balls[i]
            ball2 = balls[j]
            delta_r = ball2.r_rot - ball1.r_rot
            dist = np.linalg.norm(delta_r)
            if dist < 2 * R_BALL and dist > 0:  # Avoid division by zero
                n = delta_r / dist
                v_rel = ball1.v_rot - ball2.v_rot
                v_n = np.dot(v_rel, n)
                if v_n > 0:  # Approaching
                    m = 1  # Mass of each ball
                    J = -(1 + E) * v_n / (1 / m + 1 / m)  # Impulse magnitude
                    impulse = J * n
                    ball1.v_rot += impulse
                    ball2.v_rot -= impulse

    # Update graphics
    update_heptagon(t)
    update_balls(t)
    root.after(int(DT * 1000), update)


# --- Start Simulation ---
update()
root.mainloop()
