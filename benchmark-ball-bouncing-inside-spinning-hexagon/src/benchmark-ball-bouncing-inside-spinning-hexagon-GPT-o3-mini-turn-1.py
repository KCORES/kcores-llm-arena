import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List
import sys

# ---------------------------
# Constants and simulation parameters
# ---------------------------
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
CENTER = np.array([WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2])
FPS = 60
DT = 1 / FPS

# Heptagon parameters
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 350  # large enough to contain all balls
HEPTAGON_ANGULAR_SPEED = 2 * math.pi / 5  # full rotation (2pi) in 5 seconds

# Ball parameters
BALL_RADIUS = 15
NUM_BALLS = 20
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

# Physics parameters
GRAVITY = 500  # pixels per second^2
TRANSLATIONAL_FRICTION = 0.999  # per frame velocity decay
SPIN_FRICTION = 0.995  # per frame spin decay
RESTITUTION_BALL_WALL = 0.8  # restitution coefficient for ball-wall collisions
RESTITUTION_BALL_BALL = 0.9  # restitution coefficient for ball-ball collisions


# ---------------------------
# Helper functions for vector math
# ---------------------------
def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def rotate_vector(v: np.ndarray, angle: float) -> np.ndarray:
    """Rotate 2D vector v by angle (in radians)."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return np.array([v[0] * cos_a - v[1] * sin_a, v[0] * sin_a + v[1] * cos_a])


# ---------------------------
# Ball dataclass
# ---------------------------
@dataclass
class Ball:
    number: int
    pos: np.ndarray  # 2D position vector
    vel: np.ndarray  # 2D velocity vector
    spin_angle: float  # current spin (in radians) to draw the needle
    spin_rate: float  # angular velocity (radians per second)
    radius: float
    color: str


# ---------------------------
# Simulation class
# ---------------------------
class Simulation:
    def __init__(self, master: tk.Tk):
        self.master = master
        self.canvas = tk.Canvas(
            master, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white"
        )
        self.canvas.pack()

        # Initialize heptagon rotation angle (in radians)
        self.heptagon_rotation = 0.0

        # Create balls – all drop from the center.
        self.balls: List[Ball] = []
        for i in range(NUM_BALLS):
            # Small random offset to avoid perfect overlap initially.
            offset = np.random.uniform(-1, 1, size=2)
            ball = Ball(
                number=i + 1,
                pos=np.copy(CENTER) + offset,
                vel=np.zeros(2),
                spin_angle=0.0,
                spin_rate=np.random.uniform(-5, 5),
                radius=BALL_RADIUS,
                color=BALL_COLORS[i % len(BALL_COLORS)],
            )
            self.balls.append(ball)

        # Start simulation
        self.last_time = self.master.tk.call(
            "after", "info"
        )  # dummy to get current time
        self.animate()

    def animate(self):
        self.update_physics()
        self.draw()
        self.master.after(int(DT * 1000), self.animate)

    def update_physics(self):
        # Update heptagon rotation
        self.heptagon_rotation += HEPTAGON_ANGULAR_SPEED * DT
        self.heptagon_rotation %= 2 * math.pi

        # Pre-calculate heptagon vertices in world coordinates
        heptagon_vertices = []
        for i in range(HEPTAGON_SIDES):
            angle = 2 * math.pi * i / HEPTAGON_SIDES + self.heptagon_rotation
            vertex = (
                CENTER + np.array([math.cos(angle), math.sin(angle)]) * HEPTAGON_RADIUS
            )
            heptagon_vertices.append(vertex)

        # Update each ball: apply gravity, friction and update position
        for ball in self.balls:
            # Apply gravity (only to y component)
            ball.vel[1] += GRAVITY * DT

            # Apply friction
            ball.vel *= TRANSLATIONAL_FRICTION
            ball.spin_rate *= SPIN_FRICTION

            # Update ball position and spin
            ball.pos += ball.vel * DT
            ball.spin_angle += ball.spin_rate * DT

            # Ball-wall collision: check against each edge of the heptagon
            for i in range(HEPTAGON_SIDES):
                A = heptagon_vertices[i]
                B = heptagon_vertices[(i + 1) % HEPTAGON_SIDES]
                self.resolve_ball_wall_collision(ball, A, B)

        # Ball-ball collisions
        self.resolve_ball_ball_collisions()

    def resolve_ball_wall_collision(self, ball: Ball, A: np.ndarray, B: np.ndarray):
        # Calculate the edge vector and projection of ball center onto the edge
        edge = B - A
        edge_length2 = np.dot(edge, edge)
        if edge_length2 == 0:
            return  # avoid division by zero

        # Find projection factor t
        t = np.dot(ball.pos - A, edge) / edge_length2
        # Clamp t to [0,1] to find the closest point on the segment.
        t = max(0, min(1, t))
        closest = A + t * edge

        displacement = ball.pos - closest
        distance = np.linalg.norm(displacement)
        if distance == 0:
            # Very rare case: if ball center exactly equals closest point (e.g. at a vertex)
            displacement = ball.pos - ((A + B) / 2)
            distance = np.linalg.norm(displacement)
            if distance == 0:
                # Fallback: choose arbitrary normal
                displacement = np.array([1, 0])
                distance = 1

        if distance < ball.radius:
            # Determine inward normal for the edge.
            # Compute an edge normal (rotate edge 90 degrees) and adjust sign so that it points inward.
            normal = np.array([edge[1], -edge[0]])
            normal = normalize(normal)
            # Check direction relative to the polygon center
            if np.dot(normal, CENTER - A) < 0:
                normal = -normal

            # Separation: push the ball out of the wall to avoid penetration.
            overlap = ball.radius - distance + 0.1
            ball.pos += normal * overlap

            # Determine wall velocity at the collision point due to heptagon rotation.
            r = closest - CENTER
            # For counterclockwise rotation, the velocity is given by rotating r by 90 degrees.
            wall_vel = HEPTAGON_ANGULAR_SPEED * np.array([-r[1], r[0]])

            # Relative velocity (ball velocity relative to wall).
            v_rel = ball.vel - wall_vel
            vn = np.dot(v_rel, normal)
            if vn < 0:  # Only resolve if ball is moving toward the wall.
                # Reflect the normal component of the velocity with restitution.
                v_n_component = normal * vn
                v_t_component = v_rel - v_n_component
                # After bounce the relative normal velocity is reversed and scaled.
                v_n_component = -RESTITUTION_BALL_WALL * v_n_component
                # Optionally, a slight friction effect can damp the tangential component.
                v_t_component *= 0.98
                # Update ball velocity (add back the wall velocity).
                ball.vel = v_n_component + v_t_component + wall_vel

                # Update spin: add a small impulse proportional to the tangential speed.
                tangent = np.array([-normal[1], normal[0]])
                spin_impulse = np.dot(v_rel, tangent) * 0.05
                ball.spin_rate += spin_impulse

    def resolve_ball_ball_collisions(self):
        # Check all unique pairs for collision
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                diff = ball1.pos - ball2.pos
                dist = np.linalg.norm(diff)
                if dist == 0:
                    # To avoid division by zero, perturb slightly.
                    diff = np.random.rand(2) - 0.5
                    dist = np.linalg.norm(diff)
                if dist < ball1.radius + ball2.radius:
                    # Collision detected – separate the balls.
                    overlap = ball1.radius + ball2.radius - dist + 0.1
                    normal = diff / dist
                    ball1.pos += normal * (overlap / 2)
                    ball2.pos -= normal * (overlap / 2)

                    # Compute relative velocity in normal direction.
                    relative_vel = ball1.vel - ball2.vel
                    vn = np.dot(relative_vel, normal)
                    if vn < 0:
                        # Impulse scalar (equal mass, 1D collision along the normal)
                        impulse = -(1 + RESTITUTION_BALL_BALL) * vn / 2
                        impulse_vector = normal * impulse
                        ball1.vel += impulse_vector
                        ball2.vel -= impulse_vector

                        # Update spin: add a little spin impulse based on tangential relative speed.
                        tangent = np.array([-normal[1], normal[0]])
                        spin_impulse = np.dot(relative_vel, tangent) * 0.05
                        ball1.spin_rate += spin_impulse
                        ball2.spin_rate -= spin_impulse

    def draw(self):
        # Clear the canvas
        self.canvas.delete("all")

        # Compute heptagon vertices based on current rotation
        vertices = []
        for i in range(HEPTAGON_SIDES):
            angle = 2 * math.pi * i / HEPTAGON_SIDES + self.heptagon_rotation
            vertex = (
                CENTER + np.array([math.cos(angle), math.sin(angle)]) * HEPTAGON_RADIUS
            )
            vertices.extend(vertex.tolist())
        # Draw the heptagon (no fill, black outline)
        self.canvas.create_polygon(vertices, outline="black", fill="", width=3)

        # Draw each ball
        for ball in self.balls:
            x, y = ball.pos
            r = ball.radius
            # Draw ball circle
            self.canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
            )
            # Draw ball number (in white)
            self.canvas.create_text(
                x,
                y,
                text=str(ball.number),
                fill="white",
                font=("Helvetica", 12, "bold"),
            )
            # Draw a small needle to indicate spin direction
            needle_length = r * 0.8
            end_x = x + needle_length * math.cos(ball.spin_angle)
            end_y = y + needle_length * math.sin(ball.spin_angle)
            self.canvas.create_line(x, y, end_x, end_y, fill="black", width=2)


# ---------------------------
# Main
# ---------------------------
def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    sim = Simulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
