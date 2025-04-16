#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
20 balls bouncing in a spinning heptagon.
Allowed libs: tkinter, math, numpy, dataclasses, typing, sys.
Author: ChatGPT-o3
"""
import sys
import math
import time
from dataclasses import dataclass
from typing import List, Tuple
import tkinter as tk
import numpy as np

# --------------------------  Simulation constants ---------------------------
WIDTH, HEIGHT = 800, 800  # Canvas size
CENTER = np.array([WIDTH / 2, HEIGHT / 2], dtype=float)
HEPT_RADIUS = 250  # Radius of circumscribed circle of the heptagon
BALL_RADIUS = 15  # All balls same radius
N_BALLS = 20
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

GRAVITY = np.array([0, 500.0])  # px / s^2
DT = 1 / 60.0  # simulation step (s)
WALL_RESTITUTION = 0.8  # coefficient of restitution for wall
BALL_RESTITUTION = 0.9  # coefficient for ball-ball
FRICTION_TANGENTIAL = 0.9  # tangential damping on wall contact
LINEAR_DAMPING = 0.999  # air/friction damping each frame
SPIN_DAMPING = 0.99  # angular velocity damping each frame
OMEGA_HEPT = 2 * math.pi / 5  # heptagon angular speed (rad/s)


# --------------------------  Helper functions -------------------------------
def rotate(vec: np.ndarray, angle: float) -> np.ndarray:
    """Rotate a 2D vector by angle (radians) around origin."""
    c, s = math.cos(angle), math.sin(angle)
    return np.dot(np.array([[c, -s], [s, c]]), vec)


def wall_velocity(point: np.ndarray, angle_speed: float) -> np.ndarray:
    """Linear velocity of a point on the rotating heptagon (rigid body rotation)."""
    # v = ω x r  (in 2D: (-ω*y, ω*x))
    rel = point - CENTER
    return np.array([-angle_speed * rel[1], angle_speed * rel[0]])


# --------------------------  Data classes -----------------------------------
@dataclass
class Ball:
    idx: int
    pos: np.ndarray
    vel: np.ndarray
    omega: float  # angular velocity (rad/s) for spin
    angle: float = 0.0  # current spin angle (rad)
    color: str = "#ffffff"
    id_oval: int = 0  # Canvas id for circle
    id_text: int = 0  # Canvas id for number label

    def update_graphics(self, canvas: tk.Canvas):
        # Move the oval
        canvas.coords(
            self.id_oval,
            self.pos[0] - BALL_RADIUS,
            self.pos[1] - BALL_RADIUS,
            self.pos[0] + BALL_RADIUS,
            self.pos[1] + BALL_RADIUS,
        )
        # Update label position & rotation (Tk8.6 canvas text supports 'angle')
        canvas.coords(self.id_text, self.pos[0], self.pos[1])
        canvas.itemconfig(self.id_text, angle=math.degrees(self.angle))


# --------------------------  Physics engine ---------------------------------
class HeptagonWorld:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.balls: List[Ball] = []
        self.hept_angle = 0.0
        self.hept_id = None
        self.prev_time = time.perf_counter()
        self._create_heptagon()
        self._spawn_balls()

    # ---------------------- init helpers ------------------------------------
    def _create_heptagon(self):
        pts = self.current_hept_points()
        self.hept_id = self.canvas.create_polygon(
            *pts, outline="black", width=3, fill=""
        )
        self.canvas.tag_lower(self.hept_id)  # keep walls below balls

    def current_hept_points(self) -> List[float]:
        """Return flat list of vertex coordinates in world space (rotated)."""
        pts = []
        for k in range(7):
            angle = 2 * math.pi * k / 7 + self.hept_angle
            v = CENTER + rotate(np.array([HEPT_RADIUS, 0.0]), angle)
            pts.extend(v.tolist())
        return pts

    def _spawn_balls(self):
        rng = np.random.default_rng()
        # small jitter to prevent perfect overlap
        for i in range(N_BALLS):
            offset = rng.normal(scale=1.0, size=2) * 0.1
            b = Ball(
                idx=i + 1,
                pos=CENTER + offset,
                vel=rng.normal(scale=50.0, size=2),  # small random push
                omega=rng.normal(scale=5.0),
                color=COLORS[i],
            )
            # graphics
            b.id_oval = self.canvas.create_oval(0, 0, 0, 0, fill=b.color, outline="")
            b.id_text = self.canvas.create_text(
                0, 0, text=str(b.idx), fill="black", font=("Helvetica", 10, "bold")
            )
            b.update_graphics(self.canvas)
            self.balls.append(b)

    # ---------------------- simulation loop ---------------------------------
    def step(self):
        # compute elapsed time (in real‑time animation we fix DT, but we
        # keep clock to smoothly spin walls irrespective of frame drop)
        now = time.perf_counter()
        elapsed_real = now - self.prev_time
        self.prev_time = now
        # Advance simulation in fixed steps if > DT
        acc = elapsed_real
        while acc > 0:
            self._integrate_step(DT)
            acc -= DT
        self._update_graphics()
        self.canvas.after(int(DT * 1000), self.step)

    def _integrate_step(self, dt: float):
        # Rotate heptagon
        self.hept_angle += OMEGA_HEPT * dt
        # Ball dynamics
        for b in self.balls:
            b.vel += GRAVITY * dt
            b.pos += b.vel * dt
            b.angle += b.omega * dt
            # damping
            b.vel *= LINEAR_DAMPING
            b.omega *= SPIN_DAMPING
        self._handle_ball_wall_collisions()
        self._handle_ball_ball_collisions()

    # ---------------------- collision helpers -------------------------------
    def _hept_edges(self) -> List[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Return list of (v1, v2, inward_normal) world coords at current angle."""
        verts = []
        for k in range(7):
            angle = 2 * math.pi * k / 7 + self.hept_angle
            verts.append(CENTER + rotate(np.array([HEPT_RADIUS, 0.0]), angle))
        edges = []
        for i in range(7):
            v1, v2 = verts[i], verts[(i + 1) % 7]
            e = v2 - v1
            # outward normal is (e_y, -e_x); inward is opposite sign
            n_out = np.array([e[1], -e[0]])
            n_out /= np.linalg.norm(n_out)
            n_in = -n_out
            edges.append((v1, v2, n_in))
        return edges

    def _handle_ball_wall_collisions(self):
        edges = self._hept_edges()
        for b in self.balls:
            for v1, v2, n in edges:
                # Closest point on segment to ball center
                w = b.pos - v1
                e = v2 - v1
                L2 = np.dot(e, e)
                if L2 == 0:
                    continue
                t = max(0.0, min(1.0, np.dot(w, e) / L2))
                closest = v1 + t * e
                diff = b.pos - closest
                dist = np.linalg.norm(diff)
                if dist < BALL_RADIUS - 1e-6:  # penetrate
                    if dist == 0:
                        diff = n * 1e-6
                        dist = 1e-6
                    # push out so it exactly touches
                    penetration = BALL_RADIUS - dist
                    b.pos += n * penetration
                    # relative velocity vs wall
                    v_wall = wall_velocity(closest, OMEGA_HEPT)
                    rel_vel = b.vel - v_wall
                    vn = np.dot(rel_vel, n)
                    if vn < 0.0:
                        vt_vec = rel_vel - vn * n
                        # reflect normal component
                        rel_vel = rel_vel - (1 + WALL_RESTITUTION) * vn * n
                        # apply tangential friction
                        rel_vel -= (1 - FRICTION_TANGENTIAL) * vt_vec
                        b.vel = rel_vel + v_wall
                        # crude spin from tangential impulse
                        sign = np.cross(np.append(n, 0), np.append(vt_vec, 0))[2]
                        b.omega += 0.1 * sign

    def _handle_ball_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                a, b = self.balls[i], self.balls[j]
                diff = b.pos - a.pos
                dist = np.linalg.norm(diff)
                if dist == 0:
                    continue
                min_dist = 2 * BALL_RADIUS
                if dist < min_dist - 1e-6:
                    # push apart
                    n = diff / dist
                    penetration = (min_dist - dist) / 2
                    a.pos -= n * penetration
                    b.pos += n * penetration
                    # relative velocity along normal
                    rv = b.vel - a.vel
                    vn = np.dot(rv, n)
                    if vn < 0:
                        impulse = -(1 + BALL_RESTITUTION) * vn / 2  # equal mass
                        a.vel -= impulse * n
                        b.vel += impulse * n
                        # spin exchange (simple)
                        tang = np.array([-n[1], n[0]])
                        spin_imp = np.dot(rv, tang)
                        a.omega -= 0.05 * spin_imp
                        b.omega += 0.05 * spin_imp

    # ---------------------- drawing -----------------------------------------
    def _update_graphics(self):
        # heptagon
        self.canvas.coords(self.hept_id, *self.current_hept_points())
        for b in self.balls:
            b.update_graphics(self.canvas)


# -------------------------  main GUI ----------------------------------------
def main():
    root = tk.Tk()
    root.title("20 Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    world = HeptagonWorld(canvas)
    world.step()

    root.mainloop()


if __name__ == "__main__":
    sys.setrecursionlimit(10000)
    main()
