import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
FPS = 60
DT = 1.0 / FPS
GRAVITY = 980  # pixels/sec^2
FRICTION_COEFF = 0.75  # energy loss factor on bounce
WALL_FRICTION = 0.98  # friction when sliding along wall
BOBALL_RADIUS = 15
NUM_BALLS = 20
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

# Heptagon parameters
HEPTAGON_SIZE = 350  # radius of the circumscribed circle
ROTATION_SPEED_DEG_PER_SEC = 360 / 5  # degrees per second


@dataclass
class Ball:
    number: int
    position: np.ndarray  # [x, y]
    velocity: np.ndarray  # [vx, vy]
    radius: float
    color: str
    spin: float  # angular velocity in radians/sec
    angle: float  # current rotation angle of the ball (for rendering)


def create_heptagon_points(center_x, center_y, size):
    """Calculate vertices of a regular heptagon."""
    points = []
    for i in range(7):
        angle_deg = 360 / 7 * i
        angle_rad = math.radians(angle_deg)
        x = center_x + size * math.cos(angle_rad)
        y = center_y + size * math.sin(angle_rad)
        points.append((x, y))
    return points


class BouncingHeptagonApp:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.center = np.array([WIDTH / 2, HEIGHT / 2])
        self.heptagon_points = create_heptagon_points(
            self.center[0], self.center[1], HEPTAGON_SIZE
        )
        self.current_angle_deg = 0  # current heptagon rotation angle in degrees
        self.rotation_speed_deg = ROTATION_SPEED_DEG_PER_SEC

        # Initialize balls
        self.balls: List[Ball] = []
        for i in range(NUM_BALLS):
            color = COLORS[i % len(COLORS)]
            number = i + 1
            # initial position at the center
            pos = np.array([self.center[0], self.center[1]])
            # initial velocity: dropping vertically
            speed = 200  # pixels per sec
            vel = np.array([0, -speed])
            # spawn with a tiny random spin
            spin = np.random.uniform(-5, 5)
            ball = Ball(
                number=number,
                position=pos.copy(),
                velocity=vel,
                radius=BOBALL_RADIUS,
                color=color,
                spin=spin,
                angle=0,
            )
            self.balls.append(ball)

        self.last_time = None
        self.animate()

    def animate(self):
        now = self.master.after(0, self.tick)

    def tick(self):
        # Calculate delta time
        self.update_time()
        self.update_heptagon()
        self.update_balls()
        self.draw()
        self.master.after(int(1000 / FPS), self.tick)

    def update_time(self):
        # For calculating time difference
        if self.last_time is None:
            self.last_time = self.master.tk.call("after", "info")  # dummy
        self.current_dt = DT
        # But to get precise frame time, use time module
        # Instead, just use fixed DT for simplicity

    def update_heptagon(self):
        # Update rotation angle
        self.current_angle_deg += self.rotation_speed_deg * DT
        self.current_angle_deg = self.current_angle_deg % 360

        # Compute rotated heptagon vertices
        self.rotated_points = []
        for x, y in self.heptagon_points:
            # rotate around center
            dx = x - self.center[0]
            dy = y - self.center[1]
            r = math.hypot(dx, dy)
            theta = math.atan2(dy, dx) + math.radians(self.current_angle_deg)
            rx = self.center[0] + r * math.cos(theta)
            ry = self.center[1] + r * math.sin(theta)
            self.rotated_points.append((rx, ry))

    def update_balls(self):
        for ball in self.balls:
            # Apply gravity
            ball.velocity[1] += GRAVITY * DT

            # Update position
            ball.position += ball.velocity * DT

            # Reduce spin over time due to friction
            ball.spin *= WALL_FRICTION

            # Check collision with heptagon walls
            self.handle_wall_collision(ball)

        # Handle ball-ball collisions
        self.handle_ball_collisions()

        # Remove balls out of bounds (optional, but here just keep all)
        # Or regenerate if needed

    def handle_wall_collision(self, ball: Ball):
        # Find the closest edge and handle collision
        points = self.rotated_points
        num_points = len(points)
        for i in range(num_points):
            p1 = np.array(points[i])
            p2 = np.array(points[(i + 1) % num_points])
            collision, normal = self.check_ball_line_collision(ball, p1, p2)
            if collision:
                # Reflect velocity about normal
                v_dot_n = np.dot(ball.velocity, normal)
                if v_dot_n < 0:
                    ball.velocity -= 2 * v_dot_n * normal
                # Apply friction
                ball.velocity *= FRICTION_COEFF
                # Move the ball out of collision
                overlap = ball.radius - self.distance_point_to_segment(
                    ball.position, p1, p2
                )
                ball.position += normal * overlap

    def check_ball_line_collision(self, ball: Ball, p1, p2):
        # Check collision between ball and line segment p1-p2
        # closest point on segment to circle center
        line_vec = p2 - p1
        p1p0 = ball.position - p1
        line_len = np.dot(line_vec, line_vec)
        if line_len == 0:
            closest = p1
        else:
            t = np.dot(p1p0, line_vec) / line_len
            t = max(0, min(1, t))
            closest = p1 + t * line_vec
        dist = np.linalg.norm(ball.position - closest)
        if dist < ball.radius:
            # Compute normal vector
            normal = ball.position - closest
            if np.linalg.norm(normal) == 0:
                # In rare case center exactly on vertex, pick arbitrary normal
                normal = np.array([1, 0])
            else:
                normal = normal / np.linalg.norm(normal)
            return True, normal
        return False, None

    def distance_point_to_segment(self, p, a, b):
        # Distance from point p to segment a-b
        ap = p - a
        ab = b - a
        t = np.dot(ap, ab) / np.dot(ab, ab)
        t = max(0, min(1, t))
        closest = a + t * ab
        return np.linalg.norm(p - closest)

    def handle_ball_collisions(self):
        # Check all pairs
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                delta_pos = b2.position - b1.position
                dist = np.linalg.norm(delta_pos)
                min_dist = b1.radius + b2.radius
                if dist < min_dist and dist > 0:
                    # Calculate overlap
                    overlap = min_dist - dist
                    # Normalize delta
                    normal = delta_pos / dist
                    # Displace balls to avoid sticking
                    b1.position -= normal * (overlap / 2)
                    b2.position += normal * (overlap / 2)
                    # Relative velocity
                    rel_vel = b2.velocity - b1.velocity
                    vel_along_normal = np.dot(rel_vel, normal)
                    if vel_along_normal > 0:
                        continue  # balls moving away
                    # Calculate restitution based on friction/material
                    restitution = FRICTION_COEFF
                    # Impulse scalar
                    impulse = -(1 + restitution) * vel_along_normal
                    # Assuming equal mass
                    b1.velocity -= impulse * normal / 1
                    b2.velocity += impulse * normal / 1
                    # Optional: add rotational effects (spin) here if desired

    def draw(self):
        self.canvas.delete("all")
        # Draw heptagon
        self.draw_heptagon()
        # Draw balls
        for ball in self.balls:
            self.draw_ball(ball)
        self.canvas.update()

    def draw_heptagon(self):
        points = []
        for x, y in self.rotated_points:
            points.extend([x, y])
        self.canvas.create_polygon(points, outline="black", fill="", width=2)

    def draw_ball(self, ball: Ball):
        x, y = ball.position
        r = ball.radius
        # Draw the ball as circle
        self.canvas.create_oval(
            x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
        )
        # Draw the number at center
        self.canvas.create_text(x, y, text=str(ball.number), fill="black")
        # Optional: draw spin visual (like a small line indicating spin)
        # For visualizing spin, draw a line inside the ball pointing at current spin angle
        spin_line_length = ball.radius * 0.8
        angle = ball.angle
        line_end_x = x + spin_line_length * math.cos(angle)
        line_end_y = y + spin_line_length * math.sin(angle)
        self.canvas.create_line(x, y, line_end_x, line_end_y, fill="black", width=2)
        # Update ball's angle for spin visualization
        ball.angle += ball.spin * DT


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls inside a Spinning Heptagon")
    app = BouncingHeptagonApp(root)
    root.mainloop()
