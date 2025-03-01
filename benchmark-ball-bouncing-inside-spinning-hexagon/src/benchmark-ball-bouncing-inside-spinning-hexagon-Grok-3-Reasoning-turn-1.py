import tkinter as tk
from math import sin, cos, pi
import numpy as np
from dataclasses import dataclass

# Constants
N_BALLS = 20
R_HEPTAGON = 300  # Heptagon circumradius (pixels)
R_BALL = 10  # Ball radius (pixels)
G = 980  # Gravity (pixels/s^2)
E = 0.8  # Coefficient of restitution (bounciness)
MU = 0.3  # Coefficient of friction
OMEGA = 2 * pi / 5  # Angular velocity of heptagon (radians/s, 360° in 5s)
DT = 0.01  # Time step (s)
DRAG = 0.1  # Drag coefficient for linear and angular velocity
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

# Set up the Tkinter window and canvas
root = tk.Tk()
root.title("Bouncing Balls in Spinning Heptagon")
canvas = tk.Canvas(root, width=800, height=600)
canvas.pack()
cx, cy = 400, 300  # Center of the heptagon (canvas coordinates)


# Ball class to store properties
@dataclass
class Ball:
    x: float  # x-position (pixels)
    y: float  # y-position (pixels)
    vx: float  # x-velocity (pixels/s)
    vy: float  # y-velocity (pixels/s)
    omega: float  # Angular velocity (radians/s)
    theta: float  # Orientation angle (radians)
    color: str  # Ball color
    number: int  # Ball number (1-20)


# Initialize balls at the center with zero initial velocity
balls = [Ball(cx, cy, 0, 0, 0, 0, COLORS[i], i + 1) for i in range(N_BALLS)]

# Create heptagon lines (will be updated each frame)
heptagon_lines = [canvas.create_line(0, 0, 0, 0) for _ in range(7)]

# Create ball ovals and text labels
ball_items = []
for ball in balls:
    oval = canvas.create_oval(
        ball.x - R_BALL,
        ball.y - R_BALL,
        ball.x + R_BALL,
        ball.y + R_BALL,
        fill=ball.color,
    )
    text = canvas.create_text(
        ball.x, ball.y, text=str(ball.number), font=("Arial", 12), angle=0
    )
    ball_items.append((oval, text))

# Initial rotation angle of the heptagon
theta = 0


# Function to compute heptagon vertices based on current rotation
def get_heptagon_vertices(cx, cy, R, theta):
    vertices = []
    for k in range(7):
        angle = 2 * pi * k / 7 + theta
        x = cx + R * cos(angle)
        y = cy + R * sin(angle)
        vertices.append((x, y))
    return vertices


# Ray-casting algorithm to check if a point is inside the heptagon
def point_inside_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


# Find intersection point of two line segments
def line_intersection(line1, line2):
    x1, y1 = line1[0]
    x2, y2 = line1[1]
    x3, y3 = line2[0]
    x4, y4 = line2[1]
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denom == 0:  # Parallel lines
        return None
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom
    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_x = x1 + t * (x2 - x1)
        intersection_y = y1 + t * (y2 - y1)
        return (intersection_x, intersection_y)
    return None


# Main update function for physics and rendering
def update():
    global theta
    # Update heptagon rotation
    theta += OMEGA * DT
    vertices = get_heptagon_vertices(cx, cy, R_HEPTAGON, theta)

    # Update each ball's physics
    for ball in balls:
        # Apply gravity (y increases downward)
        ball.vy += G * DT
        # Apply drag to linear and angular velocities
        ball.vx *= 1 - DRAG * DT
        ball.vy *= 1 - DRAG * DT
        ball.omega *= 1 - DRAG * DT
        # Tentative new position
        p_tentative = (ball.x + ball.vx * DT, ball.y + ball.vy * DT)

        if point_inside_polygon(p_tentative, vertices):
            ball.x, ball.y = p_tentative
        else:
            # Check collision with each wall
            for i in range(7):
                wall = [vertices[i], vertices[(i + 1) % 7]]
                intersection = line_intersection([(ball.x, ball.y), p_tentative], wall)
                if intersection:
                    p_collide = intersection
                    # Compute normal and tangent vectors
                    v = (wall[1][0] - wall[0][0], wall[1][1] - wall[0][1])
                    n = (-v[1], v[0])
                    mag = (n[0] ** 2 + n[1] ** 2) ** 0.5
                    n = (n[0] / mag, n[1] / mag)
                    # Ensure normal points inward
                    center_to_p1 = (cx - wall[0][0], cy - wall[0][1])
                    if n[0] * center_to_p1[0] + n[1] * center_to_p1[1] < 0:
                        n = (-n[0], -n[1])
                    t = (v[0] / mag, v[1] / mag)
                    # Wall velocity due to rotation
                    r = (p_collide[0] - cx, p_collide[1] - cy)
                    v_wall = (-OMEGA * r[1], OMEGA * r[0])
                    v_ball = (ball.vx, ball.vy)
                    v_rel = (v_ball[0] - v_wall[0], v_ball[1] - v_wall[1])
                    v_rel_n_mag = v_rel[0] * n[0] + v_rel[1] * n[1]
                    if v_rel_n_mag < 0:  # Ball moving toward wall
                        J_n = -(1 + E) * v_rel_n_mag  # Normal impulse (mass = 1)
                        v_rel_t = (
                            v_rel[0] - v_rel_n_mag * n[0],
                            v_rel[1] - v_rel_n_mag * n[1],
                        )
                        # Tangential impulse due to friction
                        if v_rel_t[0] ** 2 + v_rel_t[1] ** 2 > 0:
                            unit_v_rel_t = (
                                v_rel_t[0] / (v_rel_t[0] ** 2 + v_rel_t[1] ** 2) ** 0.5,
                                v_rel_t[1] / (v_rel_t[0] ** 2 + v_rel_t[1] ** 2) ** 0.5,
                            )
                            J_t = (
                                -MU * abs(J_n) * unit_v_rel_t[0],
                                -MU * abs(J_n) * unit_v_rel_t[1],
                            )
                        else:
                            J_t = (0, 0)
                        # Update linear velocity
                        ball.vx += J_n * n[0] + J_t[0]
                        ball.vy += J_n * n[1] + J_t[1]
                        # Update angular velocity (I = (2/5) * m * r^2, m=1)
                        ball.omega += -(5 / (2 * R_BALL)) * (
                            n[0] * J_t[1] - n[1] * J_t[0]
                        )
            ball.x, ball.y = p_tentative  # Approximate position update

    # Handle ball-ball collisions
    for i in range(N_BALLS):
        for j in range(i + 1, N_BALLS):
            ball1 = balls[i]
            ball2 = balls[j]
            dx = ball2.x - ball1.x
            dy = ball2.y - ball1.y
            d = (dx**2 + dy**2) ** 0.5
            if d < 2 * R_BALL and d > 0:  # Avoid division by zero
                n = (dx / d, dy / d)
                v_rel = (ball2.vx - ball1.vx, ball2.vy - ball1.vy)
                v_rel_n = v_rel[0] * n[0] + v_rel[1] * n[1]
                if v_rel_n < 0:  # Balls moving toward each other
                    J = -(1 + E) * v_rel_n / 2  # Impulse (m1 = m2 = 1)
                    ball1.vx -= J * n[0]
                    ball1.vy -= J * n[1]
                    ball2.vx += J * n[0]
                    ball2.vy += J * n[1]

    # Update ball rotations
    for ball in balls:
        ball.theta += ball.omega * DT

    # Update display
    for i in range(7):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % 7]
        canvas.coords(heptagon_lines[i], p1[0], p1[1], p2[0], p2[1])
    for i, ball in enumerate(balls):
        oval, text = ball_items[i]
        canvas.coords(
            oval, ball.x - R_BALL, ball.y - R_BALL, ball.x + R_BALL, ball.y + R_BALL
        )
        canvas.coords(text, ball.x, ball.y)
        canvas.itemconfig(text, angle=(ball.theta * 180 / pi) % 360)

    # Schedule the next update
    root.after(int(DT * 1000), update)


# Start the simulation
update()
root.mainloop()
