import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# 20 ball colors as specified
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

BALL_COUNT = 20
BALL_RADIUS = 22
BALL_MASS = 1.0
HEPTAGON_RADIUS = 270  # Center to vertex
CANVAS_SIZE = 700
FPS = 60
DT = 1 / FPS

# Physics constants
GRAVITY = 1100.0  # pixels/s^2
FRICTION = 0.85  # Coefficient for energy lost on bounce
AIR_FRICTION = 0.995  # for in-air movement (approx.)
SPIN_FRICTION = 0.993  # for rotation lost per frame
BOUNCE_THRESHOLD = BALL_RADIUS  # Min bounce, but not more than hept radius
MAX_BOUNCE = HEPTAGON_RADIUS * 0.95  # Prevent balls from leaving

# Heptagon
HEPTAGON_SIDES = 7
HEPTAGON_ROTATION_SPEED = 72  # degrees per second


@dataclass
class Ball:
    idx: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    mass: float
    omega: float  # angular velocity (radians/sec)
    angle: float  # current rotation (radians)

    def move(self):
        self.vx *= AIR_FRICTION
        self.vy += GRAVITY * DT
        self.x += self.vx * DT
        self.y += self.vy * DT

        self.angle += self.omega * DT
        self.omega *= SPIN_FRICTION

    def energy(self):
        return self.vx**2 + self.vy**2


def heptagon_vertices(cx, cy, R, theta):  # theta in radians
    verts = []
    for i in range(HEPTAGON_SIDES):
        angle = theta + 2 * math.pi * i / HEPTAGON_SIDES
        px = cx + R * math.cos(angle)
        py = cy + R * math.sin(angle)
        verts.append((px, py))
    return verts


def point_to_line_dist(px, py, x1, y1, x2, y2):
    # get closest point on segment (x1,y1)-(x2,y2) to (px,py)
    line_mag = (x2 - x1) ** 2 + (y2 - y1) ** 2
    if line_mag == 0:
        return math.hypot(px - x1, py - y1), x1, y1, 0
    u = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_mag
    u_clamped = min(1, max(0, u))
    closest_x = x1 + u_clamped * (x2 - x1)
    closest_y = y1 + u_clamped * (y2 - y1)
    dist = math.hypot(closest_x - px, closest_y - py)
    return dist, closest_x, closest_y, u_clamped


def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def reflect(vx, vy, nx, ny):
    # Reflect vector (vx, vy) about normal (nx, ny)
    n_len = math.hypot(nx, ny)
    nx /= n_len
    ny /= n_len
    v_dot_n = vx * nx + vy * ny
    rx = vx - 2 * v_dot_n * nx
    ry = vy - 2 * v_dot_n * ny
    return rx, ry


class BouncingBallsHeptagon:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(
            self.root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="#eee"
        )
        self.canvas.pack()
        self.center = CANVAS_SIZE // 2, CANVAS_SIZE // 2
        self.time = 0.0
        self.theta = 0.0

        self.balls: List[Ball] = []
        # Place all balls in exact center with barely different velocities
        for i in range(BALL_COUNT):
            angle = 2 * math.pi * i / BALL_COUNT
            speed = np.random.uniform(50, 250)
            vx = speed * math.cos(angle)
            vy = speed * math.sin(angle) * 0.2 - 200  # Give upward kick
            color = BALL_COLORS[i]
            self.balls.append(
                Ball(
                    idx=i + 1,
                    x=self.center[0],
                    y=self.center[1],
                    vx=vx,
                    vy=vy,
                    radius=BALL_RADIUS,
                    color=color,
                    mass=BALL_MASS,
                    omega=np.random.uniform(-1.5, 1.5),  # spin
                    angle=0.0,
                )
            )
        self.running = True
        self.run()

    def run(self):
        if not self.running:
            return
        self.time += DT
        self.theta = (HEPTAGON_ROTATION_SPEED * self.time) % 360
        self.update_physics()
        self.redraw()
        self.root.after(int(1000 * DT), self.run)

    def update_physics(self):
        # Move and collide
        for ball in self.balls:
            ball.move()

        self.handle_collisions()

    def handle_collisions(self):
        # Ball-Ball collisions
        for i, b1 in enumerate(self.balls):
            for j in range(i + 1, len(self.balls)):
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist = math.hypot(dx, dy)
                min_dist = b1.radius + b2.radius
                if dist < min_dist and dist > 0.01:
                    # Overlap: separate balls
                    overlap = min_dist - dist
                    nx = dx / dist
                    ny = dy / dist
                    # Move balls apart by half overlap each
                    b1.x -= nx * overlap / 2
                    b1.y -= ny * overlap / 2
                    b2.x += nx * overlap / 2
                    b2.y += ny * overlap / 2
                    # Relative velocity
                    dvx = b2.vx - b1.vx
                    dvy = b2.vy - b1.vy
                    rel_vel = dvx * nx + dvy * ny
                    if rel_vel < 0:  # they're coming together
                        # Elastic with restitution
                        restitution = FRICTION
                        impulse = (-(1 + restitution) * rel_vel) / (
                            1 / b1.mass + 1 / b2.mass
                        )
                        ix = impulse * nx
                        iy = impulse * ny
                        b1.vx -= ix / b1.mass
                        b1.vy -= iy / b1.mass
                        b2.vx += ix / b2.mass
                        b2.vy += iy / b2.mass
                        # spin based on tangential velocity
                        tangent = (-ny, nx)
                        tangent_vel = dvx * tangent[0] + dvy * tangent[1]
                        b1.omega -= tangent_vel * 0.01
                        b2.omega += tangent_vel * 0.01

        # Ball-Heptagon wall collisions
        R = HEPTAGON_RADIUS
        theta_rad = math.radians(self.theta)
        verts = heptagon_vertices(self.center[0], self.center[1], R, theta_rad)
        for ball in self.balls:
            bpx, bpy = ball.x, ball.y
            for i in range(HEPTAGON_SIDES):
                x1, y1 = verts[i]
                x2, y2 = verts[(i + 1) % HEPTAGON_SIDES]
                dist, cx, cy, u = point_to_line_dist(bpx, bpy, x1, y1, x2, y2)
                # Check if closest point is within edge segment, and is penetration
                if dist < ball.radius - 1e-2 and 0 <= u <= 1:
                    # Compute normal
                    edge_dx = x2 - x1
                    edge_dy = y2 - y1
                    edge_len = math.hypot(edge_dx, edge_dy)
                    nx = bpy - cy
                    ny = -(bpx - cx)
                    # Flip outwards so normal points away from center
                    testx = cx + nx
                    testy = cy + ny
                    if (testx - self.center[0]) ** 2 + (testy - self.center[1]) ** 2 > (
                        bpx - self.center[0]
                    ) ** 2 + (bpy - self.center[1]) ** 2:
                        nx, ny = -nx, -ny
                    N_len = math.hypot(nx, ny)
                    if N_len < 1e-6:
                        continue
                    nx /= N_len
                    ny /= N_len
                    # Push ball outside
                    pen = ball.radius - dist + 1e-2
                    ball.x += nx * pen
                    ball.y += ny * pen

                    # Compute wall rotation & velocity at that edge: perpendicular (tangential) to radius
                    wall_omega = HEPTAGON_ROTATION_SPEED * math.pi / 180  # rad/sec
                    # Velocity of the wall at point (cx,cy) = omega * |r| = wall_omega * R
                    rel_cx = cx - self.center[0]
                    rel_cy = cy - self.center[1]
                    wall_vx = -wall_omega * rel_cy
                    wall_vy = wall_omega * rel_cx
                    # Relative velocity
                    rvx = ball.vx - wall_vx
                    rvy = ball.vy - wall_vy
                    v_dot_n = rvx * nx + rvy * ny
                    if v_dot_n < 0:
                        # Reflect velocity, apply friction
                        J = -(1 + FRICTION) * v_dot_n
                        ball.vx += J * nx
                        ball.vy += J * ny
                        # add wall V so ball doesn't "stick"
                        ball.vx = wall_vx + (ball.vx - wall_vx) * FRICTION
                        ball.vy = wall_vy + (ball.vy - wall_vy) * FRICTION
                        # add tangential effect to spin (ball rotates more by sliding)
                        tangent = (-ny, nx)  # 90 deg counterclockwise from normal
                        tan_vel = rvx * tangent[0] + rvy * tangent[1]
                        ball.omega += tan_vel * 0.06
                        # Limit bounce height (don't let escape)
                        speed = math.hypot(ball.vx, ball.vy)
                        max_speed = math.sqrt(2 * GRAVITY * MAX_BOUNCE)
                        speed = min(speed, max_speed)
                        scale = speed / max(1e-3, math.hypot(ball.vx, ball.vy))
                        ball.vx *= scale
                        ball.vy *= scale

        # Keep balls within center if they escape (for stability)
        for ball in self.balls:
            to_center = math.hypot(ball.x - self.center[0], ball.y - self.center[1])
            if to_center > HEPTAGON_RADIUS - ball.radius:
                ang = math.atan2(ball.y - self.center[1], ball.x - self.center[0])
                ball.x = self.center[0] + (
                    HEPTAGON_RADIUS - ball.radius - 1
                ) * math.cos(ang)
                ball.y = self.center[1] + (
                    HEPTAGON_RADIUS - ball.radius - 1
                ) * math.sin(ang)
                # Reverse velocity to prevent escape
                ball.vx *= -0.55
                ball.vy *= -0.55

    def redraw(self):
        self.canvas.delete("all")
        # Draw heptagon
        R = HEPTAGON_RADIUS
        theta_rad = math.radians(self.theta)
        verts = heptagon_vertices(self.center[0], self.center[1], R, theta_rad)
        coords = []
        for x, y in verts:
            coords.extend([x, y])
        self.canvas.create_polygon(coords, outline="#555", width=4, fill="#fff9ec")
        # Balls
        for ball in self.balls:
            x0 = ball.x - ball.radius
            y0 = ball.y - ball.radius
            x1 = ball.x + ball.radius
            y1 = ball.y + ball.radius
            self.canvas.create_oval(
                x0, y0, x1, y1, fill=ball.color, outline="#555", width=2
            )
            # Number (rotated)
            # We will draw a "number" rotated inside the ball
            angle = ball.angle % (2 * np.pi)
            num = str(ball.idx)
            # Choose a font size based on radius
            font_size = max(12, int(ball.radius * 1.1))
            # Center position
            cx = ball.x
            cy = ball.y
            # For drawing rotated text, precompute points
            r = ball.radius * 0.55
            text_angle = angle
            tx = cx + r * math.cos(text_angle)
            ty = cy + r * math.sin(text_angle)
            # Use canvas.create_text with anchor center
            self.canvas.create_text(
                tx,
                ty,
                text=num,
                font=("Helvetica", font_size, "bold"),
                fill="#222",
                angle=-math.degrees(angle),
            )
            # Draw spin indicator (little line from center)
            lx = cx + (ball.radius - 6) * math.cos(angle)
            ly = cy + (ball.radius - 6) * math.sin(angle)
            self.canvas.create_line(cx, cy, lx, ly, fill="#444", width=2)


if __name__ == "__main__":
    app = BouncingBallsHeptagon()
    app.root.mainloop()
