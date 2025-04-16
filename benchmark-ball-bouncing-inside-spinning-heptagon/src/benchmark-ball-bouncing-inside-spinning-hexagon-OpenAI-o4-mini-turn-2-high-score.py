import sys
import math
import tkinter as tk
import numpy as np
from dataclasses import dataclass
from typing import List

# Simulation parameters
WIDTH, HEIGHT = 800, 800
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
BALL_RADIUS = 15.0
HEP_RADIUS = 350.0
GRAVITY = np.array([0.0, 500.0])  # px/s^2 downward
DT = 0.016  # s per frame (~60fps)
WALL_RESTITUTION = 0.9
BALL_RESTITUTION = 0.9
LINEAR_FRICTION = 0.005  # simple drag
SPIN_FRICTION = 0.5  # per second
HEP_SPIN_RATE = 2 * math.pi / 5  # rad/s (360°/5s)

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


@dataclass
class Ball:
    pos: np.ndarray  # 2D position
    vel: np.ndarray  # 2D velocity
    radius: float
    color: str
    number: int
    spin_ang: float  # orientation of spin-axis arrow
    spin_vel: float  # spin angular speed


class Heptagon:
    def __init__(self, center: np.ndarray, radius: float):
        self.center = center
        self.radius = radius
        self.n = 7
        self.angle = 0.0

    def update(self, dt: float):
        self.angle = (self.angle + HEP_SPIN_RATE * dt) % (2 * math.pi)

    def vertices(self) -> List[np.ndarray]:
        verts = []
        for i in range(self.n):
            theta = self.angle + 2 * math.pi * i / self.n
            x = self.center[0] + self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            verts.append(np.array([x, y]))
        return verts

    def edges_normals(self):
        verts = self.vertices()
        edges = []
        normals = []
        for i in range(self.n):
            p1 = verts[i]
            p2 = verts[(i + 1) % self.n]
            e = p2 - p1
            # inward normal = rotate e by +90° => (-e.y, e.x)
            n = np.array([-e[1], e[0]])
            norm = np.linalg.norm(n)
            if norm > 0:
                n /= norm
            # make sure it points inward
            if np.dot(n, self.center - p1) < 0:
                n = -n
            edges.append((p1, p2))
            normals.append(n)
        return list(zip(edges, normals))


class Simulation:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.hep = Heptagon(CENTER, HEP_RADIUS)
        self.balls: List[Ball] = []
        self._create_balls()
        self._running = True
        self._after_id = None
        self.step()

    def _create_balls(self):
        for i in range(20):
            # drop from center with small jitter
            jitter = np.random.uniform(-1, 1, 2)
            pos = CENTER + jitter
            vel = np.zeros(2)
            ball = Ball(
                pos=pos,
                vel=vel,
                radius=BALL_RADIUS,
                color=BALL_COLORS[i],
                number=i + 1,
                spin_ang=0.0,
                spin_vel=0.0,
            )
            self.balls.append(ball)

    def step(self):
        if not self._running:
            return
        self._physics(DT)
        self._draw()
        self._after_id = self.canvas.after(int(DT * 1000), self.step)

    def _physics(self, dt: float):
        # 1) Update heptagon rotation
        self.hep.update(dt)
        # 2) Apply gravity & friction, integrate velocity & position
        for b in self.balls:
            b.vel += GRAVITY * dt
            b.vel *= 1 - LINEAR_FRICTION
            b.spin_vel *= 1 - SPIN_FRICTION * dt
            b.spin_ang += b.spin_vel * dt
            b.pos += b.vel * dt

        # 3) Ball–ball collisions
        n = len(self.balls)
        for i in range(n):
            for j in range(i + 1, n):
                bi, bj = self.balls[i], self.balls[j]
                dp = bj.pos - bi.pos
                dist = np.linalg.norm(dp)
                if dist == 0:
                    continue
                overlap = bi.radius + bj.radius - dist
                if overlap > 0:
                    # normalize
                    nrm = dp / dist
                    # relative velocity
                    vr = bi.vel - bj.vel
                    vn = np.dot(vr, nrm)
                    if vn < 0:
                        # impulse
                        J = -(1 + BALL_RESTITUTION) * vn / 2
                        bi.vel += J * nrm
                        bj.vel -= J * nrm
                    # separate
                    shift = nrm * (overlap / 2 + 1e-3)
                    bi.pos -= shift
                    bj.pos += shift

        # 4) Ball–wall collisions
        for b in self.balls:
            for (p1, p2), nrm in self.hep.edges_normals():
                # distance from ball center to edge
                to_p1 = b.pos - p1
                edge = p2 - p1
                edge_len2 = np.dot(edge, edge)
                # projection
                t = max(0.0, min(1.0, np.dot(to_p1, edge) / edge_len2))
                proj = p1 + t * edge
                distv = b.pos - proj
                d = np.linalg.norm(distv)
                if d < b.radius:
                    # penetration
                    if d == 0:
                        continue
                    # velocity of wall at contact point
                    # r = proj - center
                    r = proj - self.hep.center
                    # wall angular vel vector k*z
                    v_wall = np.array([-HEP_SPIN_RATE * r[1], HEP_SPIN_RATE * r[0]])
                    # relative velocity
                    v_rel = b.vel - v_wall
                    # normal component
                    vn = np.dot(v_rel, nrm)
                    if vn < 0:
                        # reflect normal
                        v_rel_n = -WALL_RESTITUTION * vn * nrm
                        # tangential
                        vt = v_rel - vn * nrm
                        # friction on tangential
                        vt *= 1 - LINEAR_FRICTION
                        # new vel
                        b.vel = v_wall + v_rel_n + vt
                        # update spin from tangential impulse approx
                        tan = np.array([nrm[1], -nrm[0]])
                        imp_t = np.dot(v_rel, tan)
                        b.spin_vel += imp_t / (b.radius + 1e-6)
                    # separate
                    correction = (b.radius - d + 1e-3) * nrm
                    b.pos += correction

    def _draw(self):
        c = self.canvas
        c.delete("all")
        # draw heptagon
        verts = self.hep.vertices()
        pts = []
        for v in verts:
            pts.extend((v[0], v[1]))
        c.create_polygon(pts, outline="black", fill="", width=3)
        # draw balls
        for b in self.balls:
            x, y = b.pos
            r = b.radius
            c.create_oval(x - r, y - r, x + r, y + r, fill=b.color, outline="black")
            # number
            c.create_text(
                x, y, text=str(b.number), fill="white", font=("Helvetica", 12, "bold")
            )
            # spin arrow
            dx = 0.8 * r * math.cos(b.spin_ang)
            dy = 0.8 * r * math.sin(b.spin_ang)
            c.create_line(x, y, x + dx, y + dy, fill="white", width=2)


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("20 Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.sim = Simulation(self.canvas)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()

    def on_close(self):
        self.sim._running = False
        if self.sim._after_id:
            self.canvas.after_cancel(self.sim._after_id)
        self.root.destroy()


if __name__ == "__main__":
    App()
