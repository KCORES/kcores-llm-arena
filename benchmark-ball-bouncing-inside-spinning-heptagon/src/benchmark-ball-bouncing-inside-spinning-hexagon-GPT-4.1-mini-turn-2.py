import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 900, 900
CENTER = np.array([WIDTH / 2, HEIGHT / 2])
FPS = 60
DT = 1 / FPS

NUM_BALLS = 20
BALL_RADIUS = 15  # radius of each ball in pixels

# Heptagon (7-sided polygon) parameters
NUM_SIDES = 7
# Spin speed: 360 degrees per 5 seconds -> 72 degrees/s in radians
SPIN_SPEED = 2 * math.pi / 5  # radians per second (angular velocity)

# Colors for balls from #f8b862 to #d66a35 (given)
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

# Gravity and friction parameters
GRAVITY = np.array([0, 980.0])  # pixels per second² downward
# We pick a strong gravity to produce clear bounce

FRICTION_COEFF = 0.95  # velocity dampening per second (will be applied per frame)
ROT_FRICTION_COEFF = 0.93

# Bounce coefficient:
# Bounce height between ball radius and heptagon radius
# Ball radius = 15
# Heptagon radius will be ~350 (chosen below)
# We will tune restitution coefficient accordingly.
# Since height of bounce ~ (vy_initial²)/(2g)
# We limit vy_initial so max height < hept_radius but > ball radius.

# For simplicity we uniformly set restitution coefficient (elasticity) to 0.8
# We will clamp the velocity after bounce if needed to fit limits.

# Heptagon radius (distance from center to vertex)
HEPTAGON_RADIUS = 350


def rotate_point(p: np.ndarray, angle: float, origin: np.ndarray = None) -> np.ndarray:
    """Rotate point p around origin by angle radians."""
    if origin is None:
        origin = np.array([0, 0])
    s, c = math.sin(angle), math.cos(angle)
    p_rel = p - origin
    rotated = np.array([p_rel[0] * c - p_rel[1] * s, p_rel[0] * s + p_rel[1] * c])
    return rotated + origin


@dataclass
class Ball:
    idx: int
    pos: np.ndarray  # 2D position vector
    vel: np.ndarray  # 2D velocity vector
    radius: float
    color: str
    angle: float  # orientation angle for spin in radians
    angular_vel: float  # angular velocity (spin)
    number: int  # number on ball

    def apply_gravity(self):
        self.vel += GRAVITY * DT

    def apply_friction(self):
        self.vel *= FRICTION_COEFF**DT
        self.angular_vel *= ROT_FRICTION_COEFF**DT

    def update(self):
        self.pos += self.vel * DT
        self.angle += self.angular_vel * DT

    def draw(self, canvas: tk.Canvas):
        x, y = self.pos
        r = self.radius

        # Draw circle
        canvas.create_oval(
            x - r, y - r, x + r, y + r, fill=self.color, outline="black", width=1
        )
        # Draw the number rotated with ball spin
        # Let's draw the number as text at center rotated by ball angle
        # Tkinter text cannot be rotated easily, so approximate with offset arcs or a dot progression
        # Instead, we draw the number upright, plus a small spin indicator dot on circumference

        # Number text upright at center
        canvas.create_text(
            x,
            y,
            text=str(self.number),
            fill="black",
            font=("Helvetica", int(r * 1.2), "bold"),
        )

        # Draw small dot indicating spin direction on circumference
        dot_angle = self.angle
        dot_x = x + r * 0.7 * math.cos(dot_angle)
        dot_y = y + r * 0.7 * math.sin(dot_angle)
        dot_r = max(2, r * 0.15)
        canvas.create_oval(
            dot_x - dot_r, dot_y - dot_r, dot_x + dot_r, dot_y + dot_r, fill="black"
        )


@dataclass
class Wall:
    p1: np.ndarray
    p2: np.ndarray

    def direction(self) -> np.ndarray:
        return self.p2 - self.p1

    def normal(self) -> np.ndarray:
        d = self.direction()
        n = np.array([-d[1], d[0]])
        return n / np.linalg.norm(n)

    def distance_to_point(self, p: np.ndarray) -> float:
        # Distance from point p to line defined by p1-p2
        n = self.normal()
        return np.dot(p - self.p1, n)

    def project_point(self, p: np.ndarray) -> np.ndarray:
        # Projection of point p onto line p1-p2
        d = self.direction()
        d_norm_sq = np.dot(d, d)
        if d_norm_sq == 0:
            return self.p1
        t = np.dot(p - self.p1, d) / d_norm_sq
        t_clamped = min(max(t, 0), 1)
        return self.p1 + d * t_clamped

    def reflect_velocity(
        self, vel: np.ndarray, restitution: float, friction: float
    ) -> Tuple[np.ndarray, float]:
        # Reflect velocity vector off wall
        n = self.normal()
        v_norm = np.dot(vel, n)
        v_tan = vel - v_norm * n

        # Reverse and reduce normal component by restitution
        v_norm_reflected = -v_norm * restitution

        # Apply friction to tangential component (simulate friction on collision)
        v_tan_friction = v_tan * friction

        # Spin effect could come from tangential velocity at contact, but we approximate
        vel_after = v_tan_friction + v_norm_reflected * n
        return vel_after


class Simulation:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        # Initial spin angle of heptagon
        self.heptagon_angle = 0.0

        # Prepare balls list
        self.balls = []

        # Build initial balls at center with no velocity (drop from center)
        for i in range(NUM_BALLS):
            ball = Ball(
                idx=i,
                pos=CENTER.copy(),
                vel=np.zeros(2),
                radius=BALL_RADIUS,
                color=BALL_COLORS[i % len(BALL_COLORS)],
                angle=0.0,
                angular_vel=0.0,
                number=i + 1,
            )
            self.balls.append(ball)

        # Precompute static walls in local polygon space
        self.local_vertices = self.compute_heptagon_vertices(HEPTAGON_RADIUS)
        # For collision detection we need walls in global coordinates updated every frame

        # Precompute materials parameters
        self.restitution = 0.8  # bounce factor
        self.wall_friction = 0.6  # tangential velocity reduction on wall hit
        self.ball_friction = FRICTION_COEFF

        # Ball-ball collision restitution approximately same as wall
        self.ball_restitution = 0.8

        # Start the update loop
        self.root.after(int(1000 / FPS), self.update)

    def compute_heptagon_vertices(self, radius: float) -> List[np.ndarray]:
        # Computes the 7 vertices of a regular heptagon centered at (0,0),
        # first vertex at angle = -pi/2 (top) for proper upright orientation.
        vertices = []
        for i in range(NUM_SIDES):
            angle = -math.pi / 2 + i * 2 * math.pi / NUM_SIDES
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            vertices.append(np.array([x, y]))
        return vertices

    def get_rotated_vertices(self) -> List[np.ndarray]:
        # Rotate local vertices by current heptagon_angle and translate to CENTER
        return [
            rotate_point(v, self.heptagon_angle) + CENTER for v in self.local_vertices
        ]

    def get_walls(self) -> List[Wall]:
        verts = self.get_rotated_vertices()
        walls = []
        for i in range(NUM_SIDES):
            p1 = verts[i]
            p2 = verts[(i + 1) % NUM_SIDES]
            walls.append(Wall(p1, p2))
        return walls

    def ball_ball_collision(self, b1: Ball, b2: Ball):
        # Check collision and resolve
        dp = b2.pos - b1.pos
        dist = np.linalg.norm(dp)
        if dist == 0:
            dist = 0.001
            dp = np.array([0.001, 0])
        overlap = b1.radius + b2.radius - dist
        if overlap > 0:
            # Push balls apart equally
            norm = dp / dist
            shift = norm * (overlap / 2 + 0.01)
            b1.pos -= shift
            b2.pos += shift

            # Relative velocity along normal
            rel_vel = b2.vel - b1.vel
            rel_norm_vel = np.dot(rel_vel, norm)

            if rel_norm_vel < 0:
                # Compute impulse scalar
                # Assume equal mass = 1 for simplicity
                impulse = -(1 + self.ball_restitution) * rel_norm_vel / 2

                impulse_vec = impulse * norm

                b1.vel -= impulse_vec
                b2.vel += impulse_vec

                # Update spin (angular velocity) based on tangential velocity change and friction
                tangent = np.array([-norm[1], norm[0]])
                rel_tan_vel = np.dot(rel_vel, tangent)

                # Apply a friction impulse on spin (simplified)
                friction_impulse = rel_tan_vel * 0.2

                b1.angular_vel -= friction_impulse
                b2.angular_vel += friction_impulse

    def ball_wall_collision(self, ball: Ball, wall: Wall):
        # Compute closest point on wall segment to ball center
        proj = wall.project_point(ball.pos)
        dist_v = ball.pos - proj
        dist = np.linalg.norm(dist_v)
        if dist == 0:
            # Exactly on wall line segment: push out by radius in normal direction
            dist_v = wall.normal()
            dist = 1e-5

        penetration = ball.radius - dist
        if penetration > 0:
            # Move ball out along normal
            n = dist_v / dist
            ball.pos += n * penetration

            # Reflect velocity
            # Calculate relative velocity at contact:
            # v_ball + omega * r_perp (perp vector = [-ny, nx])
            contact_vel = (
                ball.vel + ball.angular_vel * np.array([-n[1], n[0]]) * ball.radius
            )

            v_n = np.dot(contact_vel, n)
            v_t = contact_vel - v_n * n

            if v_n < 0:
                # Bounce normal component and apply friction on tangential
                v_n_new = -v_n * self.restitution
                v_t_new = v_t * self.wall_friction

                contact_vel_new = v_n_new * n + v_t_new

                # Calculate new linear velocity and angular velocity:
                # Linear velocity is projection of contact_vel_new without spin part
                # angular velocity approx from tangential component difference

                ball.vel = (
                    contact_vel_new
                    - ball.angular_vel * np.array([-n[1], n[0]]) * ball.radius
                )

                # update angular velocity due to friction torque from tangential change
                dv_t = v_t_new - v_t
                # torque impulse causing angular velocity change (simplified model)
                # torque = r × F, here F along tangent direction dv_t * mass (mass=1)
                # angular velocity change proportional to torque / moment of inertia

                # Moment of inertia for solid sphere 2/5*m*r², assume m=1
                I = 0.4 * ball.radius**2

                # change in angular velocity = torque / I
                torque = ball.radius * np.cross(
                    n, dv_t
                )  # dv_t is force direction, n normal
                ball.angular_vel += torque / I

                # Clamp angular velocity for stability
                max_spin = 30
                if ball.angular_vel > max_spin:
                    ball.angular_vel = max_spin
                elif ball.angular_vel < -max_spin:
                    ball.angular_vel = -max_spin

    def limit_bounce_height(self, ball: Ball):
        # Maximum bounce height is less than heptagon radius
        # Bounce height h = vy^2 / (2g)
        # We restrict ball.vel[1] (y velocity) magnitude on bounce

        # Only check when ball moving upward (vy < 0)
        vy = ball.vel[1]
        if vy < 0:
            max_height = HEPTAGON_RADIUS * 0.9  # max bounce below radius
            min_height = ball.radius * 1.2  # min bounce above ball radius

            max_vy = -math.sqrt(2 * GRAVITY[1] * max_height)
            min_vy = -math.sqrt(2 * GRAVITY[1] * min_height)

            if vy < max_vy:
                ball.vel[1] = max_vy
            if vy > min_vy:
                ball.vel[1] = min_vy

    def update(self):
        self.canvas.delete("all")

        # Update heptagon rotation
        self.heptagon_angle += SPIN_SPEED * DT
        self.heptagon_angle %= 2 * math.pi

        walls = self.get_walls()

        # 1. Apply gravity and friction
        for ball in self.balls:
            ball.apply_gravity()
            ball.apply_friction()

        # 2. Update balls position
        for ball in self.balls:
            ball.update()

        # 3. Detect and resolve ball-ball collisions
        # Naive O(n²) approach since n=20 is small
        for i in range(NUM_BALLS):
            for j in range(i + 1, NUM_BALLS):
                self.ball_ball_collision(self.balls[i], self.balls[j])

        # 4. Detect and resolve ball-wall collisions
        for ball in self.balls:
            for wall in walls:
                self.ball_wall_collision(ball, wall)
                self.limit_bounce_height(ball)

        # Draw heptagon
        verts = self.get_rotated_vertices()
        coords = []
        for v in verts:
            coords.extend(v.tolist())
        self.canvas.create_polygon(coords, outline="black", fill="", width=3)

        # Draw balls
        for ball in self.balls:
            ball.draw(self.canvas)

        # Schedule next frame
        self.root.after(int(1000 / FPS), self.update)


def main():
    root = tk.Tk()
    root.title("20 Balls Bouncing Inside a Spinning Heptagon")
    sim = Simulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
