import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 800, 800
CENTER = np.array([WIDTH // 2, HEIGHT // 2], dtype=float)
FPS = 60
DT = 1 / FPS

G = 900  # gravity pixels/s^2
FRICTION_AIR = 0.995
FRICTION_WALL = 0.85
FRICTION_BALL_SPIN = 0.995

BALL_RADIUS = 15
BALL_DIAMETER = 2 * BALL_RADIUS

HEPTAGON_RADIUS = 250  # radius of circumscribed circle around heptagon

SPIN_PERIOD = 5  # seconds per full rotation
SPIN_RPS = 1 / SPIN_PERIOD  # rotations per second
SPIN_RAD_PER_SEC = SPIN_RPS * 2 * math.pi

# Colors given, for balls 1-20 respectively
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

# The balls should bounce off with max bounce height between ball radius and heptagon radius.
# Let's define max bounce height = 0.75 * HEPTAGON_RADIUS, min bounce height = BALL_RADIUS
MAX_BOUNCE_HEIGHT = 0.75 * HEPTAGON_RADIUS
MIN_BOUNCE_HEIGHT = BALL_RADIUS


def rotate_point(p: np.ndarray, center: np.ndarray, angle_rad: float) -> np.ndarray:
    """Rotate point p around center by angle_rad radians."""
    s, c = math.sin(angle_rad), math.cos(angle_rad)
    p_rel = p - center
    p_rot = np.array([p_rel[0] * c - p_rel[1] * s, p_rel[0] * s + p_rel[1] * c])
    return center + p_rot


@dataclass
class Ball:
    number: int
    pos: np.ndarray  # np.array([x, y])
    vel: np.ndarray  # np.array([vx, vy])
    radius: float
    color: str
    spin: float  # angular velocity radians per second, positive = clockwise
    rotation: float  # current rotation angle (for drawing number spin) radians

    mass: float = 1.0

    def update(self):
        # Gravity
        self.vel[1] += G * DT

        # Air friction for velocity
        self.vel *= FRICTION_AIR

        # Spin friction
        self.spin *= FRICTION_BALL_SPIN

        # Update position
        self.pos += self.vel * DT

        # Update rotation (spin)
        self.rotation += self.spin * DT


@dataclass
class Heptagon:
    center: np.ndarray
    radius: float
    sides: int = 7
    rotation: float = 0.0  # radians, current rotation of the polygon

    def get_vertices(self) -> List[np.ndarray]:
        """Return the vertices of the heptagon, as list of np.array [x,y]"""
        vertices = []
        angle_step = 2 * math.pi / self.sides
        for i in range(self.sides):
            angle = (
                self.rotation + angle_step * i - math.pi / 2
            )  # start top vertex pointing up
            x = self.center[0] + self.radius * math.cos(angle)
            y = self.center[1] + self.radius * math.sin(angle)
            vertices.append(np.array([x, y]))
        return vertices

    def get_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Return list of edges as tuples (start, end) vertices"""
        verts = self.get_vertices()
        edges = []
        for i in range(len(verts)):
            edges.append((verts[i], verts[(i + 1) % len(verts)]))
        return edges


class Simulation:
    def __init__(self, canvas: tk.Canvas):
        self.canvas = canvas
        self.balls: List[Ball] = []
        self.heptagon = Heptagon(CENTER.copy(), HEPTAGON_RADIUS)
        self.time = 0.0

        # Initialize balls: all at center, zero velocity, some random small initial spin
        for i in range(20):
            # Start exactly at center, small random small initial velocity for spreading out
            vel_init = np.zeros(2)
            spin_init = (
                (i - 10) * 5 * math.pi / 180
            )  # some spin differences in radians/s
            ball = Ball(
                number=i + 1,
                pos=CENTER.copy(),
                vel=vel_init,
                radius=BALL_RADIUS,
                color=BALL_COLORS[i],
                spin=spin_init,
                rotation=0.0,
            )
            self.balls.append(ball)

        self.ball_ids = []  # to store canvas ids for balls drawing
        self.polygon_id = None
        self.ball_text_ids = []

    def update(self):
        # Update time and rotation of the heptagon
        self.time += DT
        self.heptagon.rotation = (SPIN_RAD_PER_SEC * self.time) % (2 * math.pi)

        edges = self.heptagon.get_edges()

        # First update balls physics without collisions
        for ball in self.balls:
            ball.update()

        # Collision with polygon edges
        for ball in self.balls:
            for start, end in edges:
                self.resolve_ball_edge_collision(ball, start, end)

        # Collision between balls
        self.resolve_ball_ball_collisions()

        # Limit bounce height by adjusting vertical velocity if too high after bounce
        # Not separately needed because ball bounces off polygon walls realistically.
        # bounce height limiting is implicit via friction and polygon size.

    def resolve_ball_edge_collision(
        self, ball: Ball, start: np.ndarray, end: np.ndarray
    ):
        """
        Reflect ball velocity and adjust position if it penetrates heptagon edges.
        """
        # Edge vector
        edge_vec = end - start
        edge_len = np.linalg.norm(edge_vec)
        edge_dir = edge_vec / edge_len

        # Normal vector pointing inside polygon:
        # polygon vertices in CCW, so normal pointing inward is perpendicular CCW (rotate edge dir by -90 degrees)
        normal = np.array([edge_dir[1], -edge_dir[0]])

        # Project ball center to the edge line
        to_ball = ball.pos - start
        proj_length = np.dot(to_ball, edge_dir)

        # Clamp the projected length to the segment
        proj_length_clamped = max(0, min(edge_len, proj_length))
        closest_point = start + proj_length_clamped * edge_dir

        displacement = ball.pos - closest_point
        dist = np.linalg.norm(displacement)

        if dist == 0:
            # Exactly on the edge point, avoid zero division
            dist = 1e-9
            displacement = normal * dist

        # Check if ball is penetrating the edge (inside polygon)
        if dist < ball.radius:
            # Penetrating, push ball outside along normal
            penetration_depth = ball.radius - dist
            push_vector = displacement / dist * penetration_depth
            ball.pos += push_vector

            # Reflect velocity along normal with friction
            v_norm = np.dot(
                ball.vel, displacement / dist
            )  # velocity component along normal

            if v_norm < 0:
                # Bounce - reverse and apply friction and energy loss limits

                # Bounce intensity dependent on material, limit bounce height:
                # velocity after bounce should be limited so max bounce height <= MAX_BOUNCE_HEIGHT

                # Calculate the bounce velocity to reach max bounce height: v = sqrt(2 * g * h)
                max_bounce_v = math.sqrt(2 * G * MAX_BOUNCE_HEIGHT)
                min_bounce_v = math.sqrt(2 * G * MIN_BOUNCE_HEIGHT)

                # Incoming normal velocity magnitude
                incoming_v = -v_norm

                # New bounce velocity magnitude between min_bounce_v and max_bounce_v
                # Scale by friction wall and reduce velocity magnitude
                new_bounce_v = min(max_bounce_v, incoming_v * FRICTION_WALL)
                new_bounce_v = max(new_bounce_v, min_bounce_v)

                # Set new normal velocity component after bounce
                v_tangent = ball.vel - (ball.vel.dot(displacement / dist)) * (
                    displacement / dist
                )
                new_norm_vec = (displacement / dist) * new_bounce_v

                ball.vel = v_tangent + new_norm_vec

                # Spin change due to collision with wall: friction slows spin (already applied globally),
                # and ball spin adjusted slightly by tangential component relative to wall normal
                # We'll slightly reverse spin proportional to tangential velocity at contact point
                rel_vel_tangent = np.dot(ball.vel, np.array([-normal[1], normal[0]]))
                ball.spin -= rel_vel_tangent * 0.0005  # empiric small effect

    def resolve_ball_ball_collisions(self):
        """Detect and respond to collisions between balls (elastic with friction effects)."""
        n = len(self.balls)
        for i in range(n):
            for j in range(i + 1, n):
                b1 = self.balls[i]
                b2 = self.balls[j]

                delta = b2.pos - b1.pos
                dist = np.linalg.norm(delta)
                if dist == 0:
                    # overlapping perfectly: perturb slightly to prevent zero division
                    dist = 1e-9
                    delta = np.array([1e-9, 0])

                if dist < b1.radius + b2.radius:
                    # Collision detected
                    penetration_depth = b1.radius + b2.radius - dist
                    norm = delta / dist

                    # Push balls apart (proportional to mass, same mass => equal push)
                    correction = norm * (penetration_depth / 2)
                    b1.pos -= correction
                    b2.pos += correction

                    # Relative velocity in normal direction
                    rel_vel = b2.vel - b1.vel
                    rel_vel_norm = np.dot(rel_vel, norm)

                    if rel_vel_norm < 0:
                        # Compute new velocities after elastic collision
                        # Using equal mass elastic collision along normal direction
                        impulse = -(1 + FRICTION_WALL) * rel_vel_norm / 2
                        impulse_vec = impulse * norm

                        b1.vel -= impulse_vec
                        b2.vel += impulse_vec

                        # Spin changes by tangential friction component between balls

                        # Relative tangential velocity at contact point
                        tangent = np.array([-norm[1], norm[0]])
                        rel_vel_tangent = np.dot(rel_vel, tangent)

                        # Change spin proportional to relative tangential velocity
                        friction_coeff = 0.01

                        b1.spin += friction_coeff * rel_vel_tangent
                        b2.spin -= friction_coeff * rel_vel_tangent

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon edges
        verts = self.heptagon.get_vertices()
        coords = []
        for v in verts:
            coords.extend(v.tolist())
        self.polygon_id = self.canvas.create_polygon(
            coords, outline="black", fill="", width=3
        )

        # Draw balls with numbers, rotated according to spin
        for ball in self.balls:
            x, y = ball.pos
            r = ball.radius

            # Draw circle
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black", width=2
            )

            # Draw rotation of number via canvas text: we draw the number as polygon, rotating it ourselves

            # Draw number with rotation
            # We'll render number on small canvas then create image rotated - complicated,
            # But tkinter doesn't support rotated text directly,
            # We'll simulate rotation by drawing number at center plus rotated offset dots around it representing spin speed

            # Instead, we'll draw the number at center with a little arrow indicating spin direction:

            # Arrow length depending on spin magnitude:
            spin_dir = -1 if ball.spin < 0 else 1
            spin_mag = min(abs(ball.spin), 10)  # clamp max display scale

            # Position for spin arrow tip
            angle = ball.rotation
            arrow_len = 10 + 5 * spin_mag

            # Arrow endpoint from center
            arrow_dx = arrow_len * math.cos(angle)
            arrow_dy = arrow_len * math.sin(angle)

            # Draw number text center line
            self.canvas.create_text(
                x,
                y,
                text=str(ball.number),
                font=("Helvetica", int(r * 1.3), "bold"),
                fill="black",
            )
            # Draw arrow indicating spin direction/strength on top of ball center
            # Arrow line
            self.canvas.create_line(
                x,
                y,
                x + spin_dir * arrow_dx,
                y + spin_dir * arrow_dy,
                fill="black",
                width=2,
                arrow=tk.LAST,
            )


def main():
    root = tk.Tk()
    root.title("20 Balls Bouncing Inside a Spinning Heptagon")

    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    sim = Simulation(canvas)

    def loop():
        sim.update()
        sim.draw()
        root.after(int(1000 / FPS), loop)

    loop()
    root.mainloop()


if __name__ == "__main__":
    main()
