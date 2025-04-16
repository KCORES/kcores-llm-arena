#!/usr/bin/env python3
import tkinter as tk
import math
import numpy as np
from typing import List


class Ball:
    def __init__(self, pos, vel, radius, color, number, spin_ang, spin_vel):
        # physics state
        self.pos = np.array(pos, dtype=float)
        self.vel = np.array(vel, dtype=float)
        self.radius = radius
        self.color = color
        self.number = number
        self.spin_ang = spin_ang
        self.spin_vel = spin_vel
        # canvas handles (assigned later)
        self.id_oval = None
        self.id_text = None
        self.id_spin_line = None


class Simulation:
    def __init__(self):
        # --- window & canvas ---
        self.width = 800
        self.height = 600
        self.center = np.array([self.width / 2, self.height / 2], dtype=float)
        self.root = tk.Tk()
        self.root.title("20 Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(
            self.root, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # --- physics parameters ---
        self.dt = 0.02  # time step (s)
        self.gravity = np.array([0, 500])  # px/s² downward
        self.friction = 0.1  # air‐like damping
        self.spin_friction = 0.2  # spin damping
        self.wall_friction = 0.2  # friction at wall contact
        self.restitution = 0.9  # bounce coefficient

        # --- heptagon params ---
        self.n_sides = 7
        # radius of circumscribed heptagon (shrink so balls never escape)
        self.ball_radius = 20
        self.poly_radius = min(self.width, self.height) / 2 - self.ball_radius - 5
        # base (unrotated) angles of the 7 vertices
        self.base_angles = [
            2 * math.pi * i / self.n_sides - math.pi / 2 for i in range(self.n_sides)
        ]
        # angular velocity: full 360° in 5s => 2π/5 rad/s
        self.angular_speed = 2 * math.pi / 5
        self.rotation_angle = 0.0

        # --- balls setup ---
        self.num_balls = 20
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
        # to “drop from the center” we scatter them in a small cloud around center
        self.jitter_range = self.poly_radius * 0.33

        # create the polygon and balls, then start
        self._make_heptagon()
        self._make_balls()
        self.running = True
        self._update()
        self.root.mainloop()

    def _make_heptagon(self):
        # initial polygon drawing
        pts = []
        for ang in self.base_angles:
            x = self.center[0] + self.poly_radius * math.cos(ang)
            y = self.center[1] + self.poly_radius * math.sin(ang)
            pts += [x, y]
        self.heptagon_id = self.canvas.create_polygon(
            pts, outline="black", fill="", width=2
        )

    def _make_balls(self):
        self.balls: List[Ball] = []
        for i in range(self.num_balls):
            # random cloud around center
            jitter = (np.random.rand(2) - 0.5) * 2 * self.jitter_range
            pos = self.center + jitter
            vel = np.array([0.0, 0.0])
            # random spin
            spin_ang = 0.0
            spin_vel = (np.random.rand() - 0.5) * 10
            ball = Ball(
                pos, vel, self.ball_radius, self.colors[i], i + 1, spin_ang, spin_vel
            )
            # draw it
            x, y = ball.pos
            r = ball.radius
            ball.id_oval = self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline=""
            )
            ball.id_text = self.canvas.create_text(
                x, y, text=str(ball.number), fill="black"
            )
            # a little line inside the ball to show its spin angle
            endx = x + math.cos(ball.spin_ang) * r * 0.6
            endy = y + math.sin(ball.spin_ang) * r * 0.6
            ball.id_spin_line = self.canvas.create_line(
                x, y, endx, endy, fill="black", width=2
            )
            self.balls.append(ball)

    def _update(self):
        if not self.running:
            return

        # --- rotate heptagon & redraw ---
        self.rotation_angle += self.angular_speed * self.dt
        verts = []
        current_verts = []
        for ang0 in self.base_angles:
            a = ang0 + self.rotation_angle
            x = self.center[0] + self.poly_radius * math.cos(a)
            y = self.center[1] + self.poly_radius * math.sin(a)
            verts += [x, y]
            current_verts.append(np.array([x, y]))
        self.canvas.coords(self.heptagon_id, *verts)

        # build edge list [(p1,p2),...]
        edges = []
        for i in range(self.n_sides):
            p1 = current_verts[i]
            p2 = current_verts[(i + 1) % self.n_sides]
            edges.append((p1, p2))

        # --- physics: gravity + damping + spin friction ---
        for b in self.balls:
            b.vel += self.gravity * self.dt
            b.vel *= 1 - self.friction * self.dt
            b.pos += b.vel * self.dt
            b.spin_vel *= 1 - self.spin_friction * self.dt
            b.spin_ang += b.spin_vel * self.dt

        # --- ball ↔ rotating‐wall collisions ---
        for b in self.balls:
            for p1, p2 in edges:
                e = p2 - p1
                L2 = e.dot(e)
                if L2 == 0:
                    continue
                # project center onto edge
                t = (b.pos - p1).dot(e) / L2
                if t < 0 or t > 1:
                    continue
                nearest = p1 + e * t
                # outward normal (vertices are CCW)
                Le = math.sqrt(L2)
                n = np.array([e[1], -e[0]]) / Le
                # signed distance from line
                d = (b.pos - p1).dot(n)
                # wall velocity at that contact point
                r_rel = nearest - self.center
                v_wall = np.array(
                    [-self.angular_speed * r_rel[1], self.angular_speed * r_rel[0]]
                )
                v_rel = b.vel - v_wall
                vn = v_rel.dot(n)
                # collision if the ball is pushing outward and its surface touches
                if d >= -b.radius and vn > 0:
                    # push inside
                    b.pos -= (d + b.radius) * n
                    # reflect normal
                    v_after = v_rel - (1 + self.restitution) * vn * n
                    # tangential friction at wall
                    vn2 = v_after.dot(n)
                    vnorm = vn2 * n
                    vtang = v_after - vnorm
                    vtang *= 1 - self.wall_friction
                    v_after = vnorm + vtang
                    # back to lab frame
                    b.vel = v_after + v_wall

        # --- ball ↔ ball collisions ---
        NB = len(self.balls)
        for i in range(NB):
            for j in range(i + 1, NB):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dp = b1.pos - b2.pos
                dist2 = dp.dot(dp)
                md = b1.radius + b2.radius
                if dist2 <= md * md:
                    dist = math.sqrt(dist2) if dist2 > 1e-6 else 1e-3
                    n = dp / dist
                    rel = b1.vel - b2.vel
                    vrn = rel.dot(n)
                    if vrn < 0:
                        # separate
                        overlap = md - dist
                        b1.pos += (overlap / 2) * n
                        b2.pos -= (overlap / 2) * n
                        # impulse
                        J = -(1 + self.restitution) * vrn / 2
                        b1.vel += J * n
                        b2.vel -= J * n

        # --- redraw balls & spin‐lines & text ---
        for b in self.balls:
            x, y = b.pos
            r = b.radius
            self.canvas.coords(b.id_oval, x - r, y - r, x + r, y + r)
            self.canvas.coords(b.id_text, x, y)
            ex = x + math.cos(b.spin_ang) * r * 0.6
            ey = y + math.sin(b.spin_ang) * r * 0.6
            self.canvas.coords(b.id_spin_line, x, y, ex, ey)

        # schedule next frame
        self.canvas.after(int(self.dt * 1000), self._update)

    def on_close(self):
        self.running = False
        self.root.destroy()


if __name__ == "__main__":
    Simulation()
