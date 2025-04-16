import tkinter as tk
import math
import random
import time
from dataclasses import dataclass
from typing import List, Tuple

# ---------------------------
# Simulation parameters
# ---------------------------
NUM_BALLS = 20
BALL_RADIUS = 15  # pixels
GRAVITY = 500.0  # pixels / s^2
LINEAR_DAMPING = 0.999  # air friction per frame
RESTITUTION = 0.8  # bounciness coefficient (<1)
TANGENTIAL_FRICTION = 0.9  # velocity loss along wall after bounce
SPIN_DAMPING = 0.995  # angular velocity loss per frame
SPIN_FACTOR = 0.2  # conversion from tangential vel to spin

HEPTAGON_RADIUS = 250  # pixels, big enough to hold balls
HEPTAGON_SIDES = 7
HEPTAGON_ROT_PERIOD = 5.0  # seconds for full rotation

CANVAS_SIZE = 700  # width and height in pixels

# Color list as provided
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

# ---------------------------
# Utility functions
# ---------------------------


def rotate_point(p: Tuple[float, float], angle: float) -> Tuple[float, float]:
    """Rotate point p=(x,y) by angle (radians) around (0,0)."""
    x, y = p
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, x * s + y * c


def world_to_canvas(x: float, y: float) -> Tuple[float, float]:
    """Convert world coordinates (0,0 at center, y-up) to canvas coords."""
    cx = CANVAS_SIZE / 2
    cy = CANVAS_SIZE / 2
    return cx + x, cy - y


def vector_length(v: Tuple[float, float]) -> float:
    x, y = v
    return math.hypot(x, y)


def normalize(v: Tuple[float, float]) -> Tuple[float, float]:
    l = vector_length(v)
    if l == 0:
        return 0.0, 0.0
    return v[0] / l, v[1] / l


def dot(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1]


# ---------------------------
# Data classes
# ---------------------------


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    angle: float  # spin orientation (radians)
    ang_vel: float  # angular velocity (rad/s)
    radius: float
    color: str
    number: int
    circle_id: int = None
    text_id: int = None

    def update_visual(self, canvas: tk.Canvas):
        # Update circle position
        x1, y1 = world_to_canvas(self.x - self.radius, self.y + self.radius)
        x2, y2 = world_to_canvas(self.x + self.radius, self.y - self.radius)
        canvas.coords(self.circle_id, x1, y1, x2, y2)
        # Update number position & rotation
        cx, cy = world_to_canvas(self.x, self.y)
        canvas.coords(self.text_id, cx, cy)
        canvas.itemconfigure(
            self.text_id, angle=-math.degrees(self.angle)
        )  # invert for screen


# ---------------------------
# Simulation world
# ---------------------------


class World:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.balls: List[Ball] = []
        self.heptagon_base_vertices = [
            (
                HEPTAGON_RADIUS * math.cos(2 * math.pi * i / HEPTAGON_SIDES),
                HEPTAGON_RADIUS * math.sin(2 * math.pi * i / HEPTAGON_SIDES),
            )
            for i in range(HEPTAGON_SIDES)
        ]
        self.heptagon_lines: List[int] = []
        self.heptagon_angle = 0.0  # current rotation angle
        self.last_time = time.time()
        self._create_balls()
        self._create_heptagon_visual()

    # -----------------
    # Initialization
    # -----------------
    def _create_balls(self):
        for i in range(NUM_BALLS):
            ball = Ball(
                x=0.0,
                y=0.0,
                vx=random.uniform(-30, 30),
                vy=random.uniform(-30, 30),
                angle=random.uniform(0, 2 * math.pi),
                ang_vel=random.uniform(-3, 3),
                radius=BALL_RADIUS,
                color=BALL_COLORS[i % len(BALL_COLORS)],
                number=i + 1,
            )
            # Visuals
            x1, y1 = world_to_canvas(ball.x - ball.radius, ball.y + ball.radius)
            x2, y2 = world_to_canvas(ball.x + ball.radius, ball.y - ball.radius)
            cid = self.canvas.create_oval(
                x1, y1, x2, y2, fill=ball.color, outline="black"
            )
            tid = self.canvas.create_text(
                *world_to_canvas(ball.x, ball.y), text=str(ball.number), fill="black"
            )
            ball.circle_id = cid
            ball.text_id = tid
            self.balls.append(ball)

    def _create_heptagon_visual(self):
        verts = self._current_heptagon_vertices()
        for i in range(HEPTAGON_SIDES):
            p1 = verts[i]
            p2 = verts[(i + 1) % HEPTAGON_SIDES]
            c1 = world_to_canvas(*p1)
            c2 = world_to_canvas(*p2)
            lid = self.canvas.create_line(*c1, *c2, fill="#555", width=3)
            self.heptagon_lines.append(lid)

    # -----------------
    # Heptagon geometry
    # -----------------
    def _current_heptagon_vertices(self) -> List[Tuple[float, float]]:
        return [
            rotate_point(v, self.heptagon_angle) for v in self.heptagon_base_vertices
        ]

    def _heptagon_edges_and_normals(self):
        verts = self._current_heptagon_vertices()
        edges = []
        normals = []
        for i in range(HEPTAGON_SIDES):
            p1 = verts[i]
            p2 = verts[(i + 1) % HEPTAGON_SIDES]
            edges.append((p1, p2))
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            # outward normal for CCW polygon
            n = (dy, -dx)
            normals.append(normalize(n))
        return edges, normals

    # -----------------
    # Physics update
    # -----------------
    def step(self, dt: float):
        # Rotate heptagon
        self.heptagon_angle += (2 * math.pi / HEPTAGON_ROT_PERIOD) * dt
        self.heptagon_angle %= 2 * math.pi
        edges, normals = self._heptagon_edges_and_normals()

        # Ball dynamics: apply gravity and integration
        for ball in self.balls:
            # Gravity
            ball.vy -= GRAVITY * dt  # y-axis up, so gravity negative
            # Integration
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            # Damp velocity
            ball.vx *= LINEAR_DAMPING
            ball.vy *= LINEAR_DAMPING

        # Ball-Wall collisions
        for ball in self.balls:
            for (p1, _), n in zip(edges, normals):
                # distance from center to edge along normal
                d = dot((ball.x - p1[0], ball.y - p1[1]), n)
                if d > -ball.radius:
                    # penetration
                    penetration = d + ball.radius
                    # push inward
                    ball.x -= penetration * n[0]
                    ball.y -= penetration * n[1]
                    # reflect velocity along normal
                    vn = dot((ball.vx, ball.vy), n)
                    vt_vec = (ball.vx - vn * n[0], ball.vy - vn * n[1])
                    ball.vx -= (1 + RESTITUTION) * vn * n[0]
                    ball.vy -= (1 + RESTITUTION) * vn * n[1]
                    # tangential friction
                    ball.vx = vt_vec[0] * TANGENTIAL_FRICTION + ball.vx
                    ball.vy = vt_vec[1] * TANGENTIAL_FRICTION + ball.vy
                    # spin update
                    tangential_speed = dot(vt_vec, (-n[1], n[0]))
                    ball.ang_vel += (tangential_speed / ball.radius) * SPIN_FACTOR

        # Ball-Ball collisions
        n_balls = len(self.balls)
        for i in range(n_balls):
            for j in range(i + 1, n_balls):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist_sq = dx * dx + dy * dy
                min_dist = b1.radius + b2.radius
                if dist_sq < min_dist * min_dist - 1e-6:
                    dist = math.sqrt(dist_sq) if dist_sq > 0 else 0.0001
                    nx, ny = dx / dist, dy / dist
                    overlap = min_dist - dist
                    # Position correction (split)
                    b1.x -= 0.5 * overlap * nx
                    b1.y -= 0.5 * overlap * ny
                    b2.x += 0.5 * overlap * nx
                    b2.y += 0.5 * overlap * ny
                    # Relative velocity
                    vrx = b2.vx - b1.vx
                    vry = b2.vy - b1.vy
                    vn = vrx * nx + vry * ny
                    if vn < 0:
                        j_imp = -(1 + RESTITUTION) * vn / 2  # equal masses
                        ix = j_imp * nx
                        iy = j_imp * ny
                        b1.vx -= ix
                        b1.vy -= iy
                        b2.vx += ix
                        b2.vy += iy
                        # simple spin due to tangential motion
                        vt = vrx * -ny + vry * nx  # tangential component
                        b1.ang_vel -= (vt / b1.radius) * SPIN_FACTOR * 0.5
                        b2.ang_vel += (vt / b2.radius) * SPIN_FACTOR * 0.5

        # Update spin and visuals
        for ball in self.balls:
            ball.ang_vel *= SPIN_DAMPING
            ball.angle += ball.ang_vel * dt
            ball.update_visual(self.canvas)

        # Update heptagon visuals
        verts = self._current_heptagon_vertices()
        for idx, lid in enumerate(self.heptagon_lines):
            p1 = verts[idx]
            p2 = verts[(idx + 1) % HEPTAGON_SIDES]
            self.canvas.coords(lid, *world_to_canvas(*p1), *world_to_canvas(*p2))

    # -----------------
    # Animation loop
    # -----------------
    def animate(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        # clamp dt to avoid big steps
        dt = min(dt, 0.03)
        self.step(dt)
        self.canvas.after(16, self.animate)


# ---------------------------
# Main
# ---------------------------


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
    canvas.pack()

    world = World(canvas)
    world.animate()

    root.mainloop()


if __name__ == "__main__":
    main()
