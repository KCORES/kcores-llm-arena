import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# window / world constants
WIDTH, HEIGHT = 800, 800
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
HEP_RADIUS = 300  # radius of heptagon
BALL_RADIUS = 15  # radius of each ball
BALL_COUNT = 20
GRAVITY = np.array([0.0, 500.0])  # pixels/sec^2 downward
LINEAR_FRICTION = 0.1  # per second
SPIN_FRICTION = 0.2  # per second
BALL_RESTITUTION = 0.9  # e for ball–ball
WALL_RESTITUTION = 0.8  # e for ball–wall
WALL_FRICTION = 0.2  # mu for ball–wall
ANG_VEL = 2 * math.pi / 5.0  # heptagon spin: 360° per 5s
DT = 1.0 / 60.0  # 60 FPS

# 20 colors as requested
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
    pos: np.ndarray  # 2D position
    vel: np.ndarray  # 2D velocity
    radius: float
    color: str
    number: int
    spin: float  # angular velocity (rad/s)
    orientation: float  # current orientation angle (rad)


class Simulation:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("20 Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        # initial rotation of the heptagon
        self.hep_angle = 0.0
        # create balls, all starting at center, zero velocity and spin
        self.balls: List[Ball] = []
        for i in range(BALL_COUNT):
            b = Ball(
                pos=CENTER.copy(),
                vel=np.zeros(2),
                radius=BALL_RADIUS,
                color=COLORS[i],
                number=i + 1,
                spin=0.0,
                orientation=0.0,
            )
            self.balls.append(b)
        # kick off animation
        self.root.after(int(DT * 1000), self.update)

    def update(self):
        """One frame of physics + redraw."""
        # advance heptagon angle
        self.hep_angle += ANG_VEL * DT

        # compute heptagon vertices
        verts = []
        for i in range(7):
            ang = self.hep_angle + 2 * math.pi * i / 7
            x = CENTER[0] + HEP_RADIUS * math.cos(ang)
            y = CENTER[1] + HEP_RADIUS * math.sin(ang)
            verts.append((x, y))

        # --- 1) apply gravity, friction, and move each ball ---
        for b in self.balls:
            b.vel += GRAVITY * DT
            b.vel *= 1.0 - LINEAR_FRICTION * DT
            b.spin *= 1.0 - SPIN_FRICTION * DT
            b.pos += b.vel * DT
            b.orientation += b.spin * DT

        # --- 2) ball–ball collisions ---
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                delta = b2.pos - b1.pos
                dist = np.linalg.norm(delta)
                if dist < b1.radius + b2.radius:
                    # push them apart
                    if dist > 1e-6:
                        normal = delta / dist
                    else:
                        normal = np.array([1.0, 0.0])
                    overlap = b1.radius + b2.radius - dist
                    b1.pos -= normal * (overlap / 2)
                    b2.pos += normal * (overlap / 2)
                    # relative velocity
                    vrel = b2.vel - b1.vel
                    vn = vrel.dot(normal)
                    if vn < 0:
                        # impulse magnitude
                        # equal mass => divide by 2 in denom
                        j = -(1 + BALL_RESTITUTION) * vn / 2.0
                        b1.vel -= j * normal
                        b2.vel += j * normal

        # --- 3) ball–wall collisions (edges + vertices) ---
        # first edges
        for b in self.balls:
            for i in range(7):
                p1 = np.array(verts[i])
                p2 = np.array(verts[(i + 1) % 7])
                self.handle_ball_wall(b, p1, p2)
        # then corners
        for b in self.balls:
            for v in verts:
                self.handle_ball_vertex(b, np.array(v))

        # --- 4) DRAW everything ---
        self.canvas.delete("all")
        # draw heptagon
        flat = [coord for v in verts for coord in v]
        self.canvas.create_polygon(*flat, outline="black", fill="", width=3)

        # draw balls
        for b in self.balls:
            x, y = b.pos
            r = b.radius
            # circle
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=b.color, outline="black"
            )
            # little radial line to show spin
            ang = b.orientation
            ex = x + math.cos(ang) * r * 0.8
            ey = y + math.sin(ang) * r * 0.8
            self.canvas.create_line(x, y, ex, ey, fill="black", width=2)
            # ball number
            self.canvas.create_text(
                x,
                y,
                text=str(b.number),
                fill="black",
                font=("Helvetica", int(r * 0.8), "bold"),
            )

        # schedule next frame
        self.root.after(int(DT * 1000), self.update)

    def handle_ball_wall(self, b: Ball, p1: np.ndarray, p2: np.ndarray):
        """Collide ball b against the line segment p1–p2."""
        w = p2 - p1
        wlen2 = w.dot(w)
        if wlen2 < 1e-8:
            return
        # projection factor t
        t = (b.pos - p1).dot(w) / wlen2
        if t < 0.0 or t > 1.0:
            return
        proj = p1 + t * w
        delta = b.pos - proj
        dist = np.linalg.norm(delta)
        if dist < b.radius:
            # push out of wall
            if dist > 1e-6:
                normal = delta / dist
            else:
                normal = np.array([1.0, 0.0])
            overlap = b.radius - dist
            b.pos += normal * overlap
            # compute wall velocity at contact
            rvec = proj - CENTER
            v_wall = ANG_VEL * np.array([-rvec[1], rvec[0]])
            # relative velocity
            vrel = b.vel - v_wall
            vn = vrel.dot(normal)
            if vn < 0:
                # normal impulse
                jn = -(1 + WALL_RESTITUTION) * vn
                # tangential direction
                tang = np.array([-normal[1], normal[0]])
                vt = vrel.dot(tang)
                # friction impulse
                jt = -WALL_FRICTION * jn * np.sign(vt)
                # apply impulses
                vrel = vrel + jn * normal + jt * tang
                b.vel = vrel + v_wall
                # spin change by torque = jt * radius
                I = 0.4 * b.radius * b.radius  # 2/5 m r^2, m=1
                if I > 1e-8:
                    b.spin += (jt * b.radius) / I

    def handle_ball_vertex(self, b: Ball, v: np.ndarray):
        """Collide ball b against a single vertex point v."""
        delta = b.pos - v
        dist = np.linalg.norm(delta)
        if dist < b.radius:
            # normal from vertex to ball
            if dist > 1e-6:
                normal = delta / dist
            else:
                normal = np.array([1.0, 0.0])
            overlap = b.radius - dist
            b.pos += normal * overlap
            # wall-vertex velocity
            rvec = v - CENTER
            v_wall = ANG_VEL * np.array([-rvec[1], rvec[0]])
            vrel = b.vel - v_wall
            vn = vrel.dot(normal)
            if vn < 0:
                jn = -(1 + WALL_RESTITUTION) * vn
                tang = np.array([-normal[1], normal[0]])
                vt = vrel.dot(tang)
                jt = -WALL_FRICTION * jn * np.sign(vt)
                vrel = vrel + jn * normal + jt * tang
                b.vel = vrel + v_wall
                I = 0.4 * b.radius * b.radius
                if I > 1e-8:
                    b.spin += (jt * b.radius) / I


if __name__ == "__main__":
    sim = Simulation()
    sim.root.mainloop()
