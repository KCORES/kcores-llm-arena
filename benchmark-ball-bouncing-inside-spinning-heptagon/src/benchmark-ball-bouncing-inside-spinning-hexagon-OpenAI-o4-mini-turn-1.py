import sys
import math
import time
import tkinter as tk
import numpy as np
from dataclasses import dataclass
from typing import List

# Simulation parameters
N_BALLS = 20
BALL_RADIUS = 15.0
HEPT_RADIUS = 250.0
CANVAS_SIZE = 2 * (HEPT_RADIUS + 20)
GRAVITY = 980.0  # px/s^2
FRICTION = 0.02  # per second
WALL_FRICTION = 0.2
RESTITUTION = 0.9
DT = 1 / 60.0
# Heptagon rotates 360°/5s => omega = 2π/5 rad/s
OMEGA = 2 * math.pi / 5.0

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


@dataclass
class Ball:
    pos: np.ndarray  # [x,y]
    vel: np.ndarray  # [vx,vy]
    angle: float  # spin angle
    ang_v: float  # spin angular vel
    color: str
    num: int
    oval_id: int = None
    text_id: int = None


class Simulation:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("20 Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(
            self.root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white"
        )
        self.canvas.pack()
        self.center = np.array([CANVAS_SIZE / 2, CANVAS_SIZE / 2])
        self.time = 0.0
        self.theta = 0.0  # current rotation angle

        # Precompute heptagon initial vertices (unrotated)
        self.base_angles = [i * 2 * math.pi / 7 for i in range(7)]

        # Create heptagon polygon on canvas
        pts = self.heptagon_points(self.theta)
        self.poly_id = self.canvas.create_polygon(
            pts, outline="black", fill="", width=3
        )

        # Create balls
        self.balls: List[Ball] = []
        for i in range(N_BALLS):
            # all start at center with tiny random velocity
            pos = self.center.copy()
            vel = np.random.randn(2) * 20
            ang_v = np.random.randn() * 5
            b = Ball(pos, vel, 0.0, ang_v, COLORS[i % len(COLORS)], i + 1)
            # draw oval and text
            x, y = pos
            r = BALL_RADIUS
            b.oval_id = self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=b.color, outline="black"
            )
            b.text_id = self.canvas.create_text(x, y, text=str(b.num), fill="black")
            self.balls.append(b)

        self.last_time = time.time()
        self.running = True
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(0, self.step)
        self.root.mainloop()

    def on_close(self):
        self.running = False
        self.root.destroy()

    def heptagon_points(self, theta):
        # rotate each base angle by theta
        pts = []
        for a in self.base_angles:
            ang = a + theta
            x = self.center[0] + HEPT_RADIUS * math.cos(ang)
            y = self.center[1] + HEPT_RADIUS * math.sin(ang)
            pts.extend((x, y))
        return pts

    def step(self):
        if not self.running:
            return
        now = time.time()
        dt = DT
        self.time += dt
        self.theta += OMEGA * dt

        # update physics
        self.update_balls(dt)
        self.handle_collisions()
        # redraw
        self.redraw()
        # schedule next
        self.root.after(int(1000 * DT), self.step)

    def update_balls(self, dt):
        for b in self.balls:
            # gravity
            b.vel[1] += GRAVITY * dt
            # simple air friction
            b.vel *= 1 - FRICTION * dt
            # spin friction
            b.ang_v *= 1 - FRICTION * dt
            # integrate
            b.pos += b.vel * dt
            b.angle += b.ang_v * dt

    def handle_collisions(self):
        # Ball-Ball
        for i in range(N_BALLS):
            for j in range(i + 1, N_BALLS):
                bi = self.balls[i]
                bj = self.balls[j]
                dp = bj.pos - bi.pos
                dist = np.linalg.norm(dp)
                min_d = 2 * BALL_RADIUS
                if dist < min_d and dist > 1e-5:
                    # normalize
                    n = dp / dist
                    # relative vel along normal
                    dv = bi.vel - bj.vel
                    vn = np.dot(dv, n)
                    if vn > 0:
                        continue
                    # impulse scalar
                    j_imp = -(1 + RESTITUTION) * vn / 2.0
                    imp = j_imp * n
                    bi.vel += imp
                    bj.vel -= imp
                    # positional correction
                    overlap = min_d - dist
                    bi.pos -= n * (overlap / 2)
                    bj.pos += n * (overlap / 2)

        # Ball-Wall (heptagon)
        pts = np.array(self.heptagon_points(self.theta)).reshape(-1, 2)
        for k in range(len(pts)):
            p1 = pts[k]
            p2 = pts[(k + 1) % len(pts)]
            edge = p2 - p1
            # inward normal: rotate edge CCW by 90
            n = np.array([-edge[1], edge[0]])
            n = n / np.linalg.norm(n)
            # ensure n points inward by checking center
            if np.dot(n, self.center - ((p1 + p2) / 2)) < 0:
                n = -n
            # for each ball
            for b in self.balls:
                # distance from ball to infinite line
                to_ball = b.pos - p1
                dist_line = np.dot(to_ball, n)
                if dist_line < BALL_RADIUS:
                    # find projection
                    proj = b.pos - dist_line * n
                    # check if proj is within segment
                    t = np.dot(proj - p1, edge) / np.dot(edge, edge)
                    if 0 <= t <= 1:
                        # collision
                        # wall point velocity: ω × r
                        r_vec = proj - self.center
                        wall_v = np.array([-OMEGA * r_vec[1], OMEGA * r_vec[0]])
                        v_rel = b.vel - wall_v
                        vn = np.dot(v_rel, n)
                        if vn < 0:
                            # reflect normal component
                            v_rel_n = -RESTITUTION * vn * n
                            # tangential component with friction
                            v_rel_t = v_rel - vn * n
                            v_rel_t *= 1 - WALL_FRICTION
                            # cap bounce height
                            vn_new = np.dot(v_rel_n, n)
                            max_vn = math.sqrt(2 * GRAVITY * HEPT_RADIUS)
                            min_vn = math.sqrt(2 * GRAVITY * BALL_RADIUS)
                            if abs(vn_new) > max_vn:
                                v_rel_n = np.sign(vn_new) * max_vn * n
                            elif abs(vn_new) < min_vn:
                                v_rel_n = np.sign(vn_new) * min_vn * n
                            # new vel
                            b.vel = v_rel_n + v_rel_t + wall_v
                            # correct position
                            b.pos += n * (BALL_RADIUS - dist_line + 1e-3)

    def redraw(self):
        # heptagon
        pts = self.heptagon_points(self.theta)
        self.canvas.coords(self.poly_id, *pts)
        # balls
        for b in self.balls:
            x, y = b.pos
            r = BALL_RADIUS
            self.canvas.coords(b.oval_id, x - r, y - r, x + r, y + r)
            self.canvas.coords(b.text_id, x, y)
            # text could be rotated by b.angle if we implement a rotated font;
            # here we simply leave it upright.


if __name__ == "__main__":
    Simulation()
