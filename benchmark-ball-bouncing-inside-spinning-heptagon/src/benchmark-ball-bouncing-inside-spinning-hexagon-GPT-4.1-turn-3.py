import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# --- Constants --- #
FPS = 60
DT = 1.0 / FPS

WIDTH, HEIGHT = 900, 900
CENTER = np.array([WIDTH // 2, HEIGHT // 2], dtype=np.float64)
HEPTAGON_RADIUS = 330  # Big enough for all balls

BALL_RADIUS = 32  # OK for 20 balls to fit
NUM_BALLS = 20

SPIN_PERIOD = 5  # seconds per full turn
SPIN_SPEED = 2 * math.pi / SPIN_PERIOD  # radians per second

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

# Physical parameters
GRAVITY = 900.0  # px/s^2
AIR_FRICTION = 0.995  # velocity loss per frame
BOUNCE_LOSS = 0.85  # coefficient of restitution (ball-wall, ball-ball)
SLIDE_FRICTION = 0.98  # angular velocity friction for balls

# --- Data Structures --- #


@dataclass
class Ball:
    idx: int  # Ball number (1-based)
    color: str
    pos: np.ndarray  # 2D position, float
    vel: np.ndarray  # 2D velocity, float
    radius: float
    omega: float  # Angular velocity (rad/s)
    theta: float  # Orientation (rad) for showing number rotation

    def move(self, dt):
        self.pos += self.vel * dt
        self.theta += self.omega * dt


# --- Heptagon Tools --- #


def get_heptagon_vertices(cx, cy, radius, angle):
    vertices = []
    for i in range(7):
        a = angle + i * 2 * math.pi / 7
        x = cx + radius * math.cos(a)
        y = cy + radius * math.sin(a)
        vertices.append(np.array([x, y], dtype=np.float64))
    return vertices


def point_in_polygon(point, verts):
    # Ray-casting algorithm
    x, y = point
    count = 0
    n = len(verts)
    for i in range(n):
        v1 = verts[i]
        v2 = verts[(i + 1) % n]
        if ((v1[1] > y) != (v2[1] > y)) and (
            x < (v2[0] - v1[0]) * (y - v1[1]) / (v2[1] - v1[1] + 1e-12) + v1[0]
        ):
            count += 1
    return count % 2 == 1


def closest_dist_to_heptagon_wall(pos, verts):
    # For a point pos, find closest point on any edge; returns edge pt, normal vector, penetration depth (positive if inside)
    min_dist = None
    min_pt = None
    normal = None
    penetration = None
    n = len(verts)
    for i in range(n):
        a = verts[i]
        b = verts[(i + 1) % n]
        ab = b - a
        ap = pos - a
        ab_len2 = np.dot(ab, ab)
        t = np.clip(np.dot(ap, ab) / ab_len2, 0, 1)
        proj = a + t * ab
        diff = pos - proj
        dist = np.linalg.norm(diff)
        edge_dir = ab / (np.linalg.norm(ab) + 1e-12)
        # Outward normal is (edge_dir[1], -edge_dir[0]) (CCW -> outward)
        nrm = np.array([edge_dir[1], -edge_dir[0]])
        # Heptagon verts are CCW, so normal points outward
        # Inside is negative cos(angle between pos-center and normal)
        wall_c2p = pos - CENTER
        if np.dot(wall_c2p, nrm) >= 0:
            nrm = -nrm  # Reverse if we're outside
        pen = -np.dot(diff, nrm)
        if min_dist is None or dist < min_dist:
            min_dist = dist
            min_pt = proj
            normal = nrm
            penetration = pen
    return min_pt, normal, penetration


# --- Ball-Ball Collision --- #
def resolve_ball_collision(b1: Ball, b2: Ball):
    dp = b2.pos - b1.pos
    dist = np.linalg.norm(dp)
    overlap = b1.radius + b2.radius - dist
    if overlap > 0 and dist > 1e-8:
        normal = dp / dist
        # Push balls apart equally
        correction = normal * (overlap / 2)
        b1.pos -= correction
        b2.pos += correction

        v1n = np.dot(b1.vel, normal)
        v2n = np.dot(b2.vel, normal)
        v1t = b1.vel - v1n * normal
        v2t = b2.vel - v2n * normal

        # Simple equal mass, 1D elastic collision along normal with bounce loss
        new_v1n = v2n * BOUNCE_LOSS
        new_v2n = v1n * BOUNCE_LOSS

        b1.vel = new_v1n * normal + v1t
        b2.vel = new_v2n * normal + v2t

        # Add a basic spin effect due to sliding at the contact
        dv = b2.vel - b1.vel
        tangent = np.array([-normal[1], normal[0]])
        rel_vel_tangent = np.dot(dv, tangent)
        spin_amount = rel_vel_tangent / (b1.radius + b2.radius)
        b1.omega -= 0.3 * spin_amount
        b2.omega += 0.3 * spin_amount


# --- Main Simulation Class --- #


class HeptagonSim:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.time = 0
        self.heptagon_angle = 0

        # Initialize balls
        self.balls: List[Ball] = []
        for i in range(NUM_BALLS):
            # All at center with tiny random velocity to start separation
            angle = 2 * math.pi * i / NUM_BALLS
            v = np.array([math.cos(angle), math.sin(angle)]) * 45.0  # px/s
            ball = Ball(
                idx=i + 1,
                color=COLORS[i % len(COLORS)],
                pos=CENTER.copy(),
                vel=v,
                radius=BALL_RADIUS,
                omega=0,
                theta=0,
            )
            self.balls.append(ball)

        self.root.after(0, self.update)

    def update(self):
        # Physics
        self.simulate()
        self.draw()
        self.time += DT
        self.heptagon_angle += SPIN_SPEED * DT
        self.root.after(int(1000 * DT), self.update)

    def simulate(self):
        # update positions and basic gravity/friction
        for ball in self.balls:
            ball.vel[1] += GRAVITY * DT
            ball.vel *= AIR_FRICTION
            ball.omega *= SLIDE_FRICTION
            ball.move(DT)

        # Ball-ball collisions
        for i in range(NUM_BALLS):
            for j in range(i + 1, NUM_BALLS):
                resolve_ball_collision(self.balls[i], self.balls[j])

        # Collisions with heptagon wall
        verts = get_heptagon_vertices(
            CENTER[0], CENTER[1], HEPTAGON_RADIUS, self.heptagon_angle
        )
        for ball in self.balls:
            min_pt, normal, penetration = closest_dist_to_heptagon_wall(ball.pos, verts)
            # If penetration>0, ball is outside. If penetration< -ball.radius, deep inside
            dist_to_wall = np.linalg.norm(ball.pos - min_pt)
            if dist_to_wall + 1e-7 > HEPTAGON_RADIUS - ball.radius:
                # Place on wall
                move_back = ball.radius - (HEPTAGON_RADIUS - dist_to_wall)
                ball.pos -= normal * move_back
                # Reflect velocity (component along normal)
                v_n = np.dot(ball.vel, normal)
                if v_n > 0:
                    ball.vel -= (1 + BOUNCE_LOSS) * v_n * normal
                # Friction: reduce tangent velocity, add angular spin
                tangent = np.array([-normal[1], normal[0]])
                v_t = np.dot(ball.vel, tangent)
                ball.vel -= 0.2 * v_t * tangent
                ball.omega += 0.13 * v_t / ball.radius
                # Don't bounce higher than the heptagon wall
                speed_limit = math.sqrt(
                    2 * GRAVITY * (HEPTAGON_RADIUS - ball.radius - 5)
                )
                if np.linalg.norm(ball.vel) > speed_limit:
                    ball.vel *= speed_limit / np.linalg.norm(ball.vel)

    def draw(self):
        self.canvas.delete("all")
        # Draw spinning heptagon
        verts = get_heptagon_vertices(
            CENTER[0], CENTER[1], HEPTAGON_RADIUS, self.heptagon_angle
        )
        flat_list = [coord for v in verts for coord in v]
        self.canvas.create_polygon(flat_list, fill="", outline="#a97b50", width=6)
        # Draw balls
        for ball in self.balls:
            x, y = ball.pos
            r = ball.radius
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="#544031", width=2
            )

            # Number with spin: rotate text by ball.theta
            angle = ball.theta
            # Place number along rotated vertical center
            text_dx = 0
            text_dy = -6
            text_x = x + text_dx * math.cos(angle) - text_dy * math.sin(angle)
            text_y = y + text_dx * math.sin(angle) + text_dy * math.cos(angle)

            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.idx),
                fill="#432a01",
                font=("Arial", int(ball.radius * 1.1), "bold"),
                angle=-math.degrees(ball.theta),  # negative for intuitive spin
            )


if __name__ == "__main__":
    import sys

    np.set_printoptions(suppress=True, precision=3)
    root = tk.Tk()
    root.title("20 Bouncing Balls in a Spinning Heptagon")
    sim = HeptagonSim(root)
    root.mainloop()
