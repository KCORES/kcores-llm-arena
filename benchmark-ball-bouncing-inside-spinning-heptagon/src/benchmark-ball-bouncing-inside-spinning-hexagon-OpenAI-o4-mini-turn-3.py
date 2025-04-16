import sys
import math
import tkinter as tk
import numpy as np
from dataclasses import dataclass
from typing import List

# === Simulation parameters ===
WIDTH, HEIGHT = 800, 800
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
N_BALLS = 20
BALL_RADIUS = 15.0
BOUNDARY_RADIUS = min(WIDTH, HEIGHT) * 0.4
GRAVITY = np.array([0.0, 300.0])  # pixels/sec^2
DT = 1 / 60.0  # seconds per frame
OMEGA = 2 * math.pi / 5.0  # spin rad/sec (360°/5s)
E_WALL = 0.9  # restitution wall
E_BALL = 0.9  # restitution ball
MU_WALL = 0.2  # tangential friction at wall
MU_AIR = 0.01  # air friction (linear & angular)
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
    pos: np.ndarray  # 2‐vector
    vel: np.ndarray  # 2‐vector
    ang: float  # rotation angle
    ang_vel: float  # angular velocity
    color: str
    number: int


def rotate_vec(v: np.ndarray, ang: float) -> np.ndarray:
    """Rotate 2D vector v by angle ang."""
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n == 0:
        return v
    return v / n


def create_balls() -> List[Ball]:
    balls = []
    for i in range(N_BALLS):
        pos = CENTER.copy()  # start at center
        vel = np.zeros(2)
        ang = 0.0
        ang_vel = (np.random.rand() * 2 - 1) * 10.0  # small random spin
        balls.append(Ball(pos, vel, ang, ang_vel, COLORS[i % len(COLORS)], i + 1))
    return balls


class HeptagonBouncer:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        # Precompute heptagon object‐space vertices (centered at 0,0)
        self.boundary_pts = []
        for k in range(7):
            ang = 2 * math.pi * k / 7
            self.boundary_pts.append(
                np.array([math.cos(ang), math.sin(ang)]) * BOUNDARY_RADIUS
            )
        self.balls = create_balls()
        self.t = 0.0
        self._draw()  # first draw
        self.master.after(int(DT * 1000), self.update)

    def update(self):
        self.t += DT
        theta = OMEGA * self.t
        # rotate boundary
        bound = [CENTER + rotate_vec(p, theta) for p in self.boundary_pts]
        # edges as pairs of points
        edges = [(bound[i], bound[(i + 1) % 7]) for i in range(7)]
        # physics
        for b in self.balls:
            # apply gravity
            b.vel += GRAVITY * DT
            # apply air friction
            b.vel *= 1 - MU_AIR
            b.ang_vel *= 1 - MU_AIR
            # move
            b.pos += b.vel * DT
            b.ang += b.ang_vel * DT
        # ball‐ball collisions
        for i in range(N_BALLS):
            for j in range(i + 1, N_BALLS):
                bi, bj = self.balls[i], self.balls[j]
                dp = bj.pos - bi.pos
                dist = np.linalg.norm(dp)
                if dist == 0:
                    continue
                if dist < 2 * BALL_RADIUS:
                    # minimum translation to separate
                    n = dp / dist
                    overlap = 2 * BALL_RADIUS - dist
                    bi.pos -= n * overlap / 2
                    bj.pos += n * overlap / 2
                    # relative velocity
                    rv = bj.vel - bi.vel
                    vn = np.dot(rv, n)
                    if vn > 0:
                        continue
                    # impulse scalar
                    j_imp = -(1 + E_BALL) * vn / 2  # m=1 for both
                    imp = j_imp * n
                    bi.vel -= imp
                    bj.vel += imp
        # ball‐wall collisions
        for b in self.balls:
            for p1, p2 in edges:
                edge = p2 - p1
                L2 = np.dot(edge, edge)
                # projection t
                t = np.dot(b.pos - p1, edge) / L2
                if t < 0 or t > 1:
                    continue
                # normal (outward)
                n = np.array([-edge[1], edge[0]])
                n = unit(n)
                # distance from wall
                d = np.dot(b.pos - p1, n)
                if d < BALL_RADIUS:
                    # push out
                    b.pos += (BALL_RADIUS - d) * n
                    vn = np.dot(b.vel, n)
                    vt = b.vel - vn * n
                    # reflect normal
                    vn_new = -E_WALL * vn
                    # friction on tangential
                    vt_new = vt * (1 - MU_WALL)
                    b.vel = vn_new * n + vt_new
                    # impart spin from tangential impulse
                    # assume impulse ~ (vt - vt_new) ⇒ Δang_vel ∝ magnitude
                    delta_vt = np.linalg.norm(vt - vt_new)
                    b.ang_vel += delta_vt / BALL_RADIUS
        # redraw
        self._draw(boundary=bound)
        self.master.after(int(DT * 1000), self.update)

    def _draw(self, boundary=None):
        self.canvas.delete("all")
        # draw heptagon
        if boundary is None:
            theta = OMEGA * self.t
            boundary = [CENTER + rotate_vec(p, theta) for p in self.boundary_pts]
        pts = []
        for p in boundary:
            pts.extend((p[0], p[1]))
        self.canvas.create_polygon(pts, outline="black", fill="", width=2)
        # draw balls
        for b in self.balls:
            x, y = b.pos
            r = BALL_RADIUS
            # circle
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=b.color, outline="black"
            )
            # spin line indicator
            # from center to radius, rotated by b.ang
            dir_line = np.array([r * 0.6, 0.0])
            end_pt = b.pos + rotate_vec(dir_line, b.ang)
            self.canvas.create_line(x, y, end_pt[0], end_pt[1], fill="black", width=2)
            # number
            self.canvas.create_text(
                x, y, text=str(b.number), fill="black", font=("Helvetica", 12, "bold")
            )


if __name__ == "__main__":
    root = tk.Tk()
    root.title("20 Balls Bouncing in a Spinning Heptagon")
    app = HeptagonBouncer(root)
    root.mainloop()
