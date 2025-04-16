import tkinter as tk
import numpy as np
import math
from dataclasses import dataclass, field
from typing import List, Tuple

# Configurations
NUM_BALLS = 20
HEPTAGON_SIDES = 7
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
CANVAS_SIZE = 820
FPS = 60
DT = 1 / FPS
GRAVITY = 800  # px/s^2
FRICTION = 0.997  # velocity loss per tick
SPIN_FRICTION = 0.985  # angular velocity loss per tick
BOUNCE_ENERGY_LOSS = 0.84  # Should make height of bounce under heptagon radius
BALL_RADIUS = 30  # px
HEP_RADIUS = 320  # px, enough for 20 balls
BALL_MASS = 1.0

# Starting center
CENTER = np.array([CANVAS_SIZE / 2, CANVAS_SIZE / 2], dtype=np.float64)


def rotate(v: np.ndarray, theta: float) -> np.ndarray:
    """Rotate vector v by theta radians."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


@dataclass
class Ball:
    idx: int  # 0-based index
    color: str
    pos: np.ndarray  # 2D position, float
    vel: np.ndarray  # 2D velocity, float
    radius: float
    mass: float = BALL_MASS
    spin: float = 0.0  # radians/sec, ball spin
    ang: float = 0.0  # ball's angle for number drawing

    def move(self, dt):
        self.pos += self.vel * dt
        self.ang += self.spin * dt
        self.vel *= FRICTION
        self.spin *= SPIN_FRICTION


@dataclass
class Heptagon:
    center: np.ndarray
    radius: float
    sides: int
    angle: float = 0.0  # current rotation angle, radians

    def get_vertices(self) -> List[np.ndarray]:
        verts = []
        for i in range(self.sides):
            theta = self.angle + (2 * np.pi * i) / self.sides
            x = self.center[0] + self.radius * np.cos(theta)
            y = self.center[1] + self.radius * np.sin(theta)
            verts.append(np.array([x, y]))
        return verts

    def get_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        verts = self.get_vertices()
        edges = []
        for i in range(self.sides):
            p1 = verts[i]
            p2 = verts[(i + 1) % self.sides]
            edges.append((p1, p2))
        return edges


class BouncingBallsApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Bouncing balls inside a spinning heptagon")
        self.canvas = tk.Canvas(
            self.root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white"
        )
        self.canvas.pack()

        self.heptagon = Heptagon(
            center=CENTER.copy(), radius=HEP_RADIUS, sides=HEPTAGON_SIDES
        )
        self.balls = [
            Ball(
                idx=i,
                color=COLORS[i % len(COLORS)],
                pos=CENTER.copy(),
                vel=np.random.randn(2) * 20 + np.array([0, 10]),
                radius=BALL_RADIUS,
                spin=np.random.uniform(-2, 2),
            )
            for i in range(NUM_BALLS)
        ]
        self.last_time = None

        self.root.after(10, self.loop)

    def loop(self):
        now = self.root.tk.call("after", "info")
        self.update()
        self.draw()
        self.root.after(int(1000 / FPS), self.loop)

    def update(self):
        # Heptagon rotates at 360deg/5s = 72deg/s = 1.2566 rad/s
        spin_speed_rad = 2 * np.pi / 5  # radians/sec
        self.heptagon.angle += spin_speed_rad * DT
        self.heptagon.angle %= 2 * np.pi

        for b in self.balls:
            b.vel[1] += GRAVITY * DT  # gravity
            b.move(DT)

        # Ball-wall collisions
        edges = self.heptagon.get_edges()
        n_edge = len(edges)
        for b in self.balls:
            # from ball to center
            bp = b.pos - self.heptagon.center
            # transform to heptagon frame
            ball_angle = (math.atan2(bp[1], bp[0]) - self.heptagon.angle) % (2 * np.pi)
            found = False
            # Which edge is closest?
            for i in range(n_edge):
                seg_angle1 = (2 * np.pi * i) / self.heptagon.sides
                seg_angle2 = (2 * np.pi * (i + 1)) / self.heptagon.sides
                # Test if angle between these edge
                diff_angle = (ball_angle - seg_angle1) % (2 * np.pi)
                edge_len = np.linalg.norm(edges[i][1] - edges[i][0])
                if 0 <= diff_angle <= (2 * np.pi) / self.heptagon.sides:
                    # Project ball position onto edge normal
                    # Get edge vector and normal
                    p1, p2 = edges[i]
                    edge = p2 - p1
                    edge_vec = edge
                    edge_normal = np.array([-edge_vec[1], edge_vec[0]])
                    edge_normal /= np.linalg.norm(edge_normal)
                    # Compute distance from ball to edge
                    ap = b.pos - p1
                    dist_to_edge = np.dot(ap, edge_normal)
                    # Find closest point on edge segment:
                    t = np.dot(ap, edge_vec) / np.dot(edge_vec, edge_vec)
                    t = min(1.0, max(0.0, t))
                    closest = p1 + t * edge_vec
                    dist_to_closest = np.linalg.norm(b.pos - closest)
                    if dist_to_edge > 0 and dist_to_closest < b.radius + 1:
                        # Collision! Slide back
                        overlap = b.radius - dist_to_closest + 1
                        n = b.pos - closest
                        if np.linalg.norm(n) != 0:
                            n = n / np.linalg.norm(n)
                        else:
                            n = edge_normal  # fallback
                        b.pos += n * overlap
                        # Project current velocity onto wall normal
                        v_n = np.dot(b.vel, n)
                        v_t = b.vel - v_n * n
                        # Wall velocity due to heptagon spin
                        rel_pt = closest - self.heptagon.center
                        wall_v = spin_speed_rad * np.array([-rel_pt[1], rel_pt[0]])
                        # Relative velocity
                        rel_vel = b.vel - wall_v
                        rel_vn = np.dot(rel_vel, n)
                        if rel_vn > 0:
                            break  # already moving away
                        v_new = b.vel - (1 + BOUNCE_ENERGY_LOSS) * rel_vn * n
                        b.vel = v_new
                        # Impart some tangential velocity from wall friction
                        b.vel = b.vel * 0.98 + wall_v * 0.02
                        # Add spin due to wall impact: torque = lever arm * impulse
                        delta_spin = 0.06 * (-np.cross(n, v_t))  # approx
                        b.spin += delta_spin
                        break
            else:
                # If outside heptagon radius, push ball back to border
                if np.linalg.norm(bp) > self.heptagon.radius - b.radius:
                    r = bp / np.linalg.norm(bp) * (self.heptagon.radius - b.radius)
                    b.pos = self.heptagon.center + r
                    # Reverse radial component of velocity
                    v_norm = bp / np.linalg.norm(bp)
                    v_n = np.dot(b.vel, v_norm)
                    if v_n > 0:
                        continue
                    b.vel -= (1 + BOUNCE_ENERGY_LOSS) * v_n * v_norm
                    b.vel *= 0.98  # friction

        # Ball-ball collisions
        for i in range(NUM_BALLS):
            for j in range(i + 1, NUM_BALLS):
                a, b = self.balls[i], self.balls[j]
                d = b.pos - a.pos
                dist = np.linalg.norm(d)
                if dist < a.radius + b.radius - 0.1:
                    if dist == 0:
                        d = np.random.randn(2)
                        dist = np.linalg.norm(d)
                    n = d / dist
                    overlap = a.radius + b.radius - dist
                    # Move both out by half overlap
                    a.pos -= n * (overlap / 2)
                    b.pos += n * (overlap / 2)
                    va_n = np.dot(a.vel, n)
                    vb_n = np.dot(b.vel, n)
                    # Conservation of momentum + simple restitution
                    va_n_new = (va_n * (a.mass - b.mass) + 2 * b.mass * vb_n) / (
                        a.mass + b.mass
                    )
                    vb_n_new = (vb_n * (b.mass - a.mass) + 2 * a.mass * va_n) / (
                        a.mass + b.mass
                    )
                    a.vel += (va_n_new - va_n) * n * BOUNCE_ENERGY_LOSS
                    b.vel += (vb_n_new - vb_n) * n * BOUNCE_ENERGY_LOSS
                    # Friction: adjust tangential velocity and spins
                    tangent = np.array([-n[1], n[0]])
                    rel_vt = np.dot(b.vel - a.vel, tangent)
                    a.vel += -tangent * rel_vt * 0.01
                    b.vel += tangent * rel_vt * 0.01
                    # angular: change spin based on tangential slip
                    a.spin += rel_vt * 0.0007
                    b.spin -= rel_vt * 0.0007

    def draw(self):
        self.canvas.delete("all")
        # Draw heptagon
        verts = self.heptagon.get_vertices()
        coords = []
        for v in verts:
            coords += [v[0], v[1]]
        self.canvas.create_polygon(coords, fill="", outline="#555", width=6)
        # Draw balls
        for b in self.balls:
            x, y = b.pos
            r = b.radius
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=b.color, outline="black"
            )
            # Draw spinning number
            theta = b.ang
            tx = x + r * 0.0
            ty = y + r * 0.0
            num = str(b.idx + 1)
            fontsize = int(b.radius * 0.95)
            # Spin the number with the ball's ang
            text_angle = theta * 180 / np.pi
            # Draw text rotated
            self._create_text_rotated(x, y, num, theta, fontsize, "black")

    def _create_text_rotated(self, x, y, text, angle, fontsize, color):
        # Draw text on a rotated canvas
        # Draw text using an image if possible (PIL), else try simple text (Tkinter doesn't rotate)
        # Here: Draw text offset from center, rotated manually
        # We'll approximate by moving the text center, not rotating the font
        # Instead, place the text a little above center, rotated according to angle
        offs = BALL_RADIUS * 0.05
        pos = rotate(np.array([0, -offs]), angle)
        self.canvas.create_text(
            x + pos[0],
            y + pos[1],
            text=text,
            fill=color,
            font=("Arial", fontsize, "bold"),
        )

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    BouncingBallsApp().run()
