import tkinter as tk
import math
import time
import sys
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
FPS = 60
GRAVITY = 980  # pixels/sec^2, scaled for visual effect
FRICTION = 0.99  # friction coefficient for damping
BOUNCE_COEFFICIENT = 0.7  # energy loss on bounce
NUM_BALLS = 20
BALL_RADIUS = 15
HEPTAGON_SIZE = 300
SPIN_SPEED_DEG_PER_SEC = 360 / 5  # 360 degrees in 5 seconds

colors_hex = [
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    color: str
    number: int
    spin: float  # in radians/sec
    angle: float  # current rotation angle of the ball (for visual spinning)
    radius: float = BALL_RADIUS
    # optional: mass could be added if needed, but assume equal mass for all


class BouncingBallsInHeptagon:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        # Initialize heptagon parameters
        self.heptagon_center = (WIDTH / 2, HEIGHT / 2)
        self.heptagon_rotation = 0.0  # in radians
        self.heptagon_rotation_speed = math.radians(
            SPIN_SPEED_DEG_PER_SEC
        )  # radians/sec

        # Generate heptagon vertices
        self.heptagon_vertices = self.compute_heptagon_vertices()

        # Initialize balls
        self.balls: List[Ball] = []
        self.init_balls()

        # Timing
        self.last_time = time.time()

        # Run main loop
        self.running = True
        self.master.protocol("WM_DELETE_WINDOW", self.on_close)
        self.animate()

    def on_close(self):
        self.running = False
        self.master.destroy()

    def compute_heptagon_vertices(self):
        vertices = []
        angle_between = 2 * math.pi / 7
        for i in range(7):
            angle = i * angle_between + self.heptagon_rotation
            x = self.heptagon_center[0] + HEPTAGON_SIZE * math.cos(angle)
            y = self.heptagon_center[1] + HEPTAGON_SIZE * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def init_balls(self):
        cx, cy = self.heptagon_center
        for i in range(NUM_BALLS):
            color = colors_hex[i % len(colors_hex)]
            number = i + 1
            # Initial position: center
            x, y = cx, cy
            # Initial velocity: zero
            vx, vy = 0.0, 0.0
            # Spin: for visual effect, can assign a small random spin
            spin = 0.0  # initial spin
            angle = 0.0
            ball = Ball(x, y, vx, vy, color, number, spin, angle)
            self.balls.append(ball)

    def animate(self):
        if not self.running:
            return

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Update heptagon rotation
        self.heptagon_rotation += self.heptagon_rotation_speed * dt
        self.heptagon_vertices = self.compute_heptagon_vertices()

        # Simulation steps
        self.update_balls(dt)
        self.handle_collisions()
        self.draw()

        # Schedule next frame
        self.master.after(int(1000 / FPS), self.animate)

    def update_balls(self, dt):
        for ball in self.balls:
            # Update velocities with gravity
            ball.vy += GRAVITY * dt

            # Apply friction on velocities
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update positions
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt

            # Apply spin (simulate some rotation per frame, for visualization)
            ball.angle += ball.spin * dt

            # Check collision with heptagon walls
            self.check_collision_with_heptagon(ball)

        # Handle ball-to-ball collisions
        self.handle_ball_collisions()

    def handle_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist = math.hypot(dx, dy)
                min_dist = b1.radius + b2.radius
                if dist < min_dist and dist > 0:
                    # Collision detected
                    # Normalize collision vector
                    nx = dx / dist
                    ny = dy / dist

                    # Relative velocity
                    dvx = b1.vx - b2.vx
                    dvy = b1.vy - b2.vy
                    # Velocity along normal
                    vn = dvx * nx + dvy * ny

                    if vn < 0:
                        # Calculate impulse scalar
                        impulse = -(1 + BOUNCE_COEFFICIENT) * vn / 2  # equally massed
                        # Apply impulse
                        b1.vx += impulse * nx
                        b1.vy += impulse * ny
                        b2.vx -= impulse * nx
                        b2.vy -= impulse * ny

                        # Positional correction to avoid sinking
                        overlap = min_dist - dist
                        correction_ratio = 0.5  # move both balls equally
                        b1.x -= overlap * nx * correction_ratio
                        b1.y -= overlap * ny * correction_ratio
                        b2.x += overlap * nx * correction_ratio
                        b2.y += overlap * ny * correction_ratio

    def check_collision_with_heptagon(self, ball):
        # For each wall, check collision
        vertices = self.heptagon_vertices
        for i in range(7):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % 7]
            # Check for collision with edge v1-v2
            self.check_collision_ball_line(ball, v1, v2)

    def check_collision_ball_line(self, ball, v1, v2):
        # Compute closest distance from ball center to line segment
        px, py = ball.x, ball.y
        x1, y1 = v1
        x2, y2 = v2
        line_dx = x2 - x1
        line_dy = y2 - y1
        length_sq = line_dx**2 + line_dy**2

        if length_sq == 0:
            # v1 and v2 are same point
            dist = math.hypot(px - x1, py - y1)
            if dist < ball.radius:
                # Reflect
                self.resolve_collision_ball_wall(ball, (x1, y1))
            return

        # Project point onto line segment
        t = ((px - x1) * line_dx + (py - y1) * line_dy) / length_sq
        t_clamped = max(0, min(1, t))
        closest_x = x1 + t_clamped * line_dx
        closest_y = y1 + t_clamped * line_dy
        dx = px - closest_x
        dy = py - closest_y
        dist = math.hypot(dx, dy)
        if dist < ball.radius:
            # collision
            # Determine normal vector
            normal = (dx / dist, dy / dist)
            # Reflect ball velocity
            self.resolve_collision_ball_wall(ball, (closest_x, closest_y), normal)

    def resolve_collision_ball_wall(self, ball, contact_point, normal=None):
        # Compute normal vector if not provided
        if normal is None:
            dx = ball.x - contact_point[0]
            dy = ball.y - contact_point[1]
            dist = math.hypot(dx, dy)
            if dist == 0:
                # ball center exactly on contact point; pick arbitrary normal
                normal = (1, 0)
            else:
                normal = (dx / dist, dy / dist)

        # Decompose velocity
        vnx = ball.vx * normal[0] + ball.vy * normal[1]
        # Reflect
        if vnx < 0:
            # invert normal component
            ball.vx -= (1 + BOUNCE_COEFFICIENT) * vnx * normal[0]
            ball.vy -= (1 + BOUNCE_COEFFICIENT) * vnx * normal[1]
            # Apply friction
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            # move ball out of collision
            overlap = ball.radius - math.hypot(
                ball.x - contact_point[0], ball.y - contact_point[1]
            )
            correction_x = normal[0] * overlap
            correction_y = normal[1] * overlap
            ball.x += correction_x
            ball.y += correction_y

    def draw(self):
        self.canvas.delete("all")
        # Draw heptagon
        self.draw_heptagon()

        # Draw balls
        for ball in self.balls:
            self.draw_ball(ball)

        self.canvas.update()

    def draw_heptagon(self):
        # draw a polygon connecting vertices
        pts = []
        for v in self.heptagon_vertices:
            pts.extend([v[0], v[1]])
        self.canvas.create_polygon(pts, outline="black", fill="", width=2)

    def draw_ball(self, ball):
        x, y = ball.x, ball.y
        r = ball.radius
        self.canvas.create_oval(
            x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
        )
        # Draw number at center
        self.canvas.create_text(x, y, text=str(ball.number), fill="black")
        # Optional: draw spin indication (like a small line showing rotation)
        line_length = ball.radius
        end_x = x + line_length * math.cos(ball.angle)
        end_y = y + line_length * math.sin(ball.angle)
        self.canvas.create_line(x, y, end_x, end_y, fill="black", width=2)


# Main execution
if __name__ == "__main__":
    root = tk.Tk()
    root.title("20 Balls Bouncing inside a Spinning Heptagon")
    app = BouncingBallsInHeptagon(root)
    root.mainloop()
