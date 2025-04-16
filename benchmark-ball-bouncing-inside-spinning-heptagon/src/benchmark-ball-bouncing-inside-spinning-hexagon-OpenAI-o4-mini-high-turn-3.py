#!/usr/bin/env python3
import sys
import math
import time
import numpy as np
import tkinter as tk
from dataclasses import dataclass

# Simulation of 20 balls bouncing under gravity inside a spinning heptagon.
# Balls collide with each other and with the rotating walls (with restitution & friction),
# spin with friction, and display their number rotated to indicate spin.


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    angle: float  # orientation angle in radians
    w: float  # angular velocity (rad/s)
    color: str
    id_oval: int = 0
    id_text: int = 0


class BouncingHeptagon:
    def __init__(self):
        # Window & canvas
        self.WIDTH = 800
        self.HEIGHT = 800
        self.root = tk.Tk()
        self.root.title("20 Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(
            self.root, width=self.WIDTH, height=self.HEIGHT, bg="white"
        )
        self.canvas.pack()

        # Container (heptagon) parameters
        self.cx = self.WIDTH / 2
        self.cy = self.HEIGHT / 2
        self.R = min(self.WIDTH, self.HEIGHT) * 0.40  # heptagon radius
        self.NW = 7  # number of walls
        # Angular speed of container: 360° per 5s
        self.omega_container = 2 * math.pi / 5.0  # rad/s
        self.theta = 0.0  # current angle

        # Ball parameters
        self.NBALL = 20
        self.ball_radius = self.R * 0.06  # ball radius ~6% of container radius
        self.g = 500.0  # gravity (px/s^2)
        self.e = 0.90  # restitution coefficient
        self.mu = 0.20  # friction coefficient at wall
        self.spin_friction = 0.50  # angular friction per second

        # Colors for the 20 balls
        self.colors = [
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

        # Create the 7 wall segments
        self.walls = []
        for i in range(self.NW):
            line = self.canvas.create_line(0, 0, 0, 0, width=2)
            self.walls.append(line)

        # Create balls, all start at center with zero translation velocity
        # but give each a nonzero spin so you can see the numbers rotate.
        self.balls = []
        for i in range(self.NBALL):
            x0 = self.cx
            y0 = self.cy
            vx0 = 0.0
            vy0 = 0.0
            ang0 = 0.0
            # deterministic “random” spin:
            w0 = math.sin(i * 1.3) * 4.0
            c = self.colors[i]
            oid = self.canvas.create_oval(
                x0 - self.ball_radius,
                y0 - self.ball_radius,
                x0 + self.ball_radius,
                y0 + self.ball_radius,
                fill=c,
                outline="",
            )
            tid = self.canvas.create_text(
                x0,
                y0,
                text=str(i + 1),
                fill="black",
                font=("Arial", int(self.ball_radius)),
                angle=0,
            )
            b = Ball(x0, y0, vx0, vy0, ang0, w0, c, oid, tid)
            self.balls.append(b)

        # Timing
        self.dt = 1 / 60.0  # fixed time step
        self.root.after(0, self.update)
        self.root.mainloop()

    def update(self):
        dt = self.dt

        # 1) Rotate container
        self.theta += self.omega_container * dt
        # compute rotated heptagon vertices
        verts = []
        for i in range(self.NW + 1):
            ang = self.theta + 2 * math.pi * i / self.NW
            x = self.cx + self.R * math.cos(ang)
            y = self.cy + self.R * math.sin(ang)
            verts.append((x, y))
        # update wall segments
        for i in range(self.NW):
            x1, y1 = verts[i]
            x2, y2 = verts[i + 1]
            self.canvas.coords(self.walls[i], x1, y1, x2, y2)

        # 2) Integrate gravity & translation
        for b in self.balls:
            b.vy += self.g * dt
            b.x += b.vx * dt
            b.y += b.vy * dt
            # spin integration
            b.angle += b.w * dt
            # spin friction
            b.w *= max(0.0, 1.0 - self.spin_friction * dt)

        # 3) Ball-ball collisions
        N = self.NBALL
        R2 = (2 * self.ball_radius) ** 2
        for i in range(N):
            bi = self.balls[i]
            for j in range(i + 1, N):
                bj = self.balls[j]
                dx = bj.x - bi.x
                dy = bj.y - bi.y
                d2 = dx * dx + dy * dy
                if d2 < R2:
                    d = math.sqrt(d2) if d2 > 1e-9 else 0.0
                    # collision normal
                    nx = dx / d if d > 0 else 1.0
                    ny = dy / d if d > 0 else 0.0
                    # push apart half/half
                    overlap = 2 * self.ball_radius - d
                    bi.x -= nx * overlap * 0.5
                    bi.y -= ny * overlap * 0.5
                    bj.x += nx * overlap * 0.5
                    bj.y += ny * overlap * 0.5
                    # relative velocity
                    rvx = bi.vx - bj.vx
                    rvy = bi.vy - bj.vy
                    vn = rvx * nx + rvy * ny
                    if vn < 0:
                        J = -(1.0 + self.e) * vn * 0.5
                        bi.vx += J * nx
                        bi.vy += J * ny
                        bj.vx -= J * nx
                        bj.vy -= J * ny

        # 4) Ball-wall collisions
        for b in self.balls:
            for i in range(self.NW):
                x1, y1 = verts[i]
                x2, y2 = verts[i + 1]
                dx, dy = x2 - x1, y2 - y1
                L2 = dx * dx + dy * dy
                # project ball center onto wall
                t = ((b.x - x1) * dx + (b.y - y1) * dy) / L2
                if t < 0:
                    px, py = x1, y1
                elif t > 1:
                    px, py = x2, y2
                else:
                    px, py = x1 + dx * t, y1 + dy * t
                ddx = b.x - px
                ddy = b.y - py
                d2 = ddx * ddx + ddy * ddy
                if d2 < self.ball_radius * self.ball_radius:
                    d = math.sqrt(d2) if d2 > 1e-9 else 0.0
                    # outward normal
                    nx = (
                        ddx / d
                        if d > 0
                        else (
                            (b.x - self.cx) / math.hypot(b.x - self.cx, b.y - self.cy)
                        )
                    )
                    ny = (
                        ddy / d
                        if d > 0
                        else (
                            (b.y - self.cy) / math.hypot(b.x - self.cx, b.y - self.cy)
                        )
                    )
                    # tangent
                    tx, ty = -ny, nx
                    # separate
                    overlap = self.ball_radius - d
                    b.x += nx * overlap
                    b.y += ny * overlap
                    # wall's linear velocity at contact
                    rx, ry = px - self.cx, py - self.cy
                    vw_x = -self.omega_container * ry
                    vw_y = self.omega_container * rx
                    # relative vel
                    rvx = b.vx - vw_x
                    rvy = b.vy - vw_y
                    vn = rvx * nx + rvy * ny
                    if vn < 0:
                        # normal bounce
                        vnew_n = -self.e * vn
                        # tangential part
                        vt = rvx * tx + rvy * ty
                        vnew_t = vt * (1.0 - self.mu)
                        # reconstruct world vel
                        b.vx = vw_x + vnew_n * nx + vnew_t * tx
                        b.vy = vw_y + vnew_n * ny + vnew_t * ty
                        # impart a bit of spin from tangential impulse
                        delta_vt = vnew_t - vt
                        b.w += delta_vt / self.ball_radius

        # 5) Draw balls
        for b in self.balls:
            x1 = b.x - self.ball_radius
            y1 = b.y - self.ball_radius
            x2 = b.x + self.ball_radius
            y2 = b.y + self.ball_radius
            self.canvas.coords(b.id_oval, x1, y1, x2, y2)
            self.canvas.coords(b.id_text, b.x, b.y)
            # rotate the number according to b.angle
            angdeg = math.degrees(b.angle) % 360
            self.canvas.itemconfig(b.id_text, angle=angdeg)

        # next frame
        self.root.after(int(dt * 1000), self.update)


if __name__ == "__main__":
    BouncingHeptagon()
