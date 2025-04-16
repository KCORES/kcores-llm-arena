import tkinter as tk
import math
import time
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 900, 900  # canvas size
CENTER = np.array([WIDTH / 2, HEIGHT / 2], dtype=float)
FPS = 60

N_SIDES = 7  # heptagon
ANGULAR_VELOCITY = 2 * math.pi / 5  # radians per second (360° per 5 sec)
# heptagon radius (distance from center to vertex)
HEPTAGON_RADIUS = 350

BALL_RADIUS = 15
BALL_COUNT = 20

GRAVITY = np.array([0, 800.0])  # pixels/sec², downward
FRICTION_AIR = 0.99
FRICTION_ROT = 0.97
FRICTION_BOUNCE = 0.7  # to limit bounce height between ball radius and heptagon radius

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

FONT = ("Consolas", 14, "bold")


@dataclass
class Ball:
    number: int
    color: str
    pos: np.ndarray  # shape (2,)
    vel: np.ndarray  # shape (2,)
    radius: float
    angle: float  # spin angle in radians (for number rotation)
    angular_velocity: float  # spin speed in rad/sec

    mass: float = 1.0  # same mass for all balls

    def update(self, dt):
        # Apply gravity
        self.vel += GRAVITY * dt
        # Apply air friction on linear velocity
        self.vel *= FRICTION_AIR
        # Update position
        self.pos += self.vel * dt
        # Apply angular friction on spin
        self.angular_velocity *= FRICTION_ROT
        # Update spin angle
        self.angle += self.angular_velocity * dt
        self.angle %= 2 * math.pi

    def draw(self, canvas):
        x, y = self.pos
        r = self.radius
        # Draw ball circle
        canvas.create_oval(
            x - r, y - r, x + r, y + r, fill=self.color, outline="black", width=1
        )

        # Draw the number on ball rotated by spin angle
        # We'll draw the number text on a rotated image-like way by calculating offset for characters.
        # Since tkinter doesn't support rotated text, approximate by drawing the number horizontally centered.
        # We'll simulate spin by offsetting the number position in a circle.

        # Offset number position with spin
        # Just visually shift the text horizontally by sin(angle)*3 px and vertically by cos(angle)*3 px,
        # for impression of spin.

        offset_x = 3 * math.sin(self.angle)
        offset_y = 3 * math.cos(self.angle)
        canvas.create_text(
            x + offset_x, y + offset_y, text=str(self.number), font=FONT, fill="black"
        )


def polygon_vertices(
    center: np.ndarray, radius: float, sides: int, rotation: float
) -> List[np.ndarray]:
    vertices = []
    for i in range(sides):
        angle = rotation + 2 * math.pi * i / sides - math.pi / 2  # start pointing up
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        vertices.append(np.array([x, y], dtype=float))
    return vertices


def edge_normals(vertices: List[np.ndarray]) -> List[np.ndarray]:
    # Normals pointing inside the polygon
    normals = []
    n = len(vertices)
    for i in range(n):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % n]
        edge = p2 - p1
        # normal to edge (outward)
        normal = np.array([-edge[1], edge[0]])
        normal /= np.linalg.norm(normal)
        normals.append(normal)
    return normals


def point_line_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    # distance from point to line segment ab
    ab = b - a
    t = np.clip(np.dot(point - a, ab) / np.dot(ab, ab), 0, 1)
    proj = a + t * ab
    return np.linalg.norm(point - proj)


class Simulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.balls: List[Ball] = []
        self.time = 0.0
        self.last_time = time.time()
        self.heptagon_rotation = 0.0
        self.create_balls()
        self.running = True

        # Precompute mass and moment of inertia for the ball (solid circle)
        # For simplicity, consider mass=1, I = 0.5 * m * r^2

    def create_balls(self):
        # All balls start exactly in center with zero velocity,
        # but to avoid initial perfect overlap, give slight upward initial velocities with some variation
        center = CENTER.copy()
        initial_speed = 60.0
        angle_base = 2 * math.pi / BALL_COUNT
        for i in range(BALL_COUNT):
            color = COLORS[i % len(COLORS)]
            # Spread initial velocities radially outwards slightly to avoid permanent overlap
            angle = angle_base * i
            vel = np.array(
                [initial_speed * math.cos(angle), initial_speed * math.sin(angle)]
            )
            ball = Ball(
                number=i + 1,
                color=color,
                pos=center.copy(),
                vel=vel,
                radius=BALL_RADIUS,
                angle=0.0,
                angular_velocity=10.0
                * (-1 if i % 2 == 0 else 1),  # alternate spin directions
            )
            self.balls.append(ball)

    def run(self):
        self.update()
        self.root.after(int(1000 / FPS), self.run)

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0.05:
            dt = 0.05  # clamp large jumps

        self.time += dt
        # Update heptagon rotation
        self.heptagon_rotation = (self.heptagon_rotation + ANGULAR_VELOCITY * dt) % (
            2 * math.pi
        )

        # Compute heptagon vertices and edges in fixed coordinates
        vertices = polygon_vertices(
            CENTER, HEPTAGON_RADIUS, N_SIDES, self.heptagon_rotation
        )
        n = len(vertices)

        # Edges: pairs of vertices
        edges = [(vertices[i], vertices[(i + 1) % n]) for i in range(n)]

        # Normal vectors, outward from polygon edges
        normals = []
        for p1, p2 in edges:
            edge = p2 - p1
            normal = np.array([-edge[1], edge[0]])
            normal /= np.linalg.norm(normal)
            normals.append(normal)

        # Update all balls (apply physics integration)
        for ball in self.balls:
            ball.update(dt)

        # Check ball-ball collisions
        self.handle_ball_collisions()

        # Check and resolve collisions with polygon edges
        for ball in self.balls:
            self.handle_edge_collision(ball, edges, normals, dt)

        self.draw(vertices)

    def handle_ball_collisions(self):
        # Basic elastic collision between balls with friction affecting bounciness
        balls = self.balls
        for i in range(len(balls)):
            b1 = balls[i]
            for j in range(i + 1, len(balls)):
                b2 = balls[j]
                delta = b2.pos - b1.pos
                dist = np.linalg.norm(delta)
                min_dist = b1.radius + b2.radius
                if dist == 0:
                    # Prevent divide by zero: nudge slightly
                    delta = np.random.rand(2) - 0.5
                    dist = np.linalg.norm(delta)
                if dist < min_dist:
                    # Resolve penetration
                    overlap = min_dist - dist
                    n = delta / dist
                    # push balls away from each other by overlap/2
                    b1.pos -= n * (overlap / 2)
                    b2.pos += n * (overlap / 2)

                    # Relative velocity
                    rel_vel = b2.vel - b1.vel
                    vel_along_n = np.dot(rel_vel, n)
                    if vel_along_n > 0:
                        continue  # balls separating

                    # Calculate impulse scalar (elastic collision with friction coefficient)
                    e = FRICTION_BOUNCE  # restitution coefficient (less than 1)
                    j = -(1 + e) * vel_along_n
                    j /= 1 / b1.mass + 1 / b2.mass

                    impulse = j * n
                    b1.vel -= impulse / b1.mass
                    b2.vel += impulse / b2.mass

                    # Spin change due to friction, rough estimation:
                    tangent = np.array([-n[1], n[0]])
                    rel_tangent_vel = np.dot(rel_vel, tangent)
                    angular_impulse = (
                        rel_tangent_vel * 0.01
                    )  # tune factor for spinning effect

                    b1.angular_velocity -= angular_impulse / b1.radius
                    b2.angular_velocity += angular_impulse / b2.radius

    def handle_edge_collision(
        self,
        ball: Ball,
        edges: List[Tuple[np.ndarray, np.ndarray]],
        normals: List[np.ndarray],
        dt,
    ):
        # Check collision and respond with polygon edge

        pos = ball.pos
        r = ball.radius
        radius_limit_upper = HEPTAGON_RADIUS  # max bounce height ~ radius of heptagon
        radius_limit_lower = (
            BALL_RADIUS + 3
        )  # min bounce height (slightly bigger than ball radius)

        # We'll check penetration and penetration correction against each edge
        # Because polygon is convex, at most one edge will be penetrated significantly.

        penetration_data = []
        for i, (p1, p2) in enumerate(edges):
            # Project ball center on edge segment to see if closest point is inside segment
            edge_vec = p2 - p1
            edge_len = np.linalg.norm(edge_vec)
            edge_dir = edge_vec / edge_len
            v = pos - p1
            proj = np.dot(v, edge_dir)

            if proj < 0:
                closest = p1
            elif proj > edge_len:
                closest = p2
            else:
                closest = p1 + edge_dir * proj

            dist = np.linalg.norm(pos - closest)
            if dist < r:
                penetration_depth = r - dist
                normal = pos - closest
                if np.linalg.norm(normal) == 0:
                    # ball center exactly at closest point, assign normal outward from center:
                    normal = pos - CENTER
                    if np.linalg.norm(normal) == 0:
                        normal = np.array([0, -1])
                normal /= np.linalg.norm(normal)
                penetration_data.append((penetration_depth, normal, closest, i))

        if not penetration_data:
            return

        # Resolve the deepest penetration among edges (most urgent collision)
        penetration_data.sort(key=lambda x: -x[0])
        penetration_depth, normal, closest_point, edge_idx = penetration_data[0]

        # Push ball outside penetration
        ball.pos += normal * penetration_depth
        # Reflect velocity relative to edge normal with friction limiting bounce
        v = ball.vel
        vn = np.dot(v, normal)
        vt = v - vn * normal
        if vn < 0:
            # restitution depending on bounce height limits

            # Calculate the bounce speed necessary to reach bounce height inside limits:
            # h = v^2/(2g), so v = sqrt(2gh)
            # Bounce height limited [radius_limit_lower, radius_limit_upper]
            max_v = math.sqrt(2 * np.linalg.norm(GRAVITY) * radius_limit_upper)
            min_v = math.sqrt(2 * np.linalg.norm(GRAVITY) * radius_limit_lower)
            bounce_v = -vn * FRICTION_BOUNCE
            if bounce_v > max_v:
                bounce_v = max_v
            elif bounce_v < min_v:
                bounce_v = min_v

            ball.vel = vt + normal * bounce_v

            # Spin change due to friction between ball and edge (surface friction slows spin)
            friction_factor = 0.85
            ball.angular_velocity *= friction_factor

            # Also, a small spin impulse depending on tangential velocity:
            tangent = np.array([-normal[1], normal[0]])
            vt_speed = np.dot(v, tangent)
            ball.angular_velocity += vt_speed * 0.05 / ball.radius

    def draw(self, vertices: List[np.ndarray]):
        self.canvas.delete("all")

        # Draw heptagon polygon
        points = []
        for v in vertices:
            points.extend(v.tolist())
        self.canvas.create_polygon(points, outline="black", fill="", width=3)

        # Draw balls
        for ball in self.balls:
            ball.draw(self.canvas)


def main():
    root = tk.Tk()
    root.title("20 Balls Bouncing in a Spinning Heptagon")
    sim = Simulation(root)
    sim.run()
    root.mainloop()


if __name__ == "__main__":
    main()
