import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    spin: float
    radius: float
    color: str
    number: int


class HeptagonBouncingBallsApp:
    """
    This class manages the tkinter canvas, the balls, and the rotating heptagon.
    It handles all updates for motion, collisions, and rendering.
    """

    def __init__(self, master: tk.Tk):
        self.master = master
        self.master.title("Heptagon Bouncing Balls")

        # Canvas dimensions
        self.width = 800
        self.height = 600

        # Heptagon setup
        self.heptagon_center = (self.width / 2, self.height / 2)
        self.heptagon_radius = 300.0  # Enough to hold all balls
        self.sides = 7
        self.rotation_angle = 0.0  # Current angle of heptagon in degrees
        self.rotation_speed = 72.0  # Degrees per second (360 deg in 5s)

        # Ball setup
        self.ball_radius = 15.0
        self.num_balls = 20
        self.gravity = 0.4  # Gravity
        self.friction = 0.99  # Velocity friction
        self.restitution = 0.9  # Collision restitution
        self.spin_friction = 0.98

        # Color list for the 20 balls
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

        self.canvas = tk.Canvas(
            self.master, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        # Create balls
        self.balls: List[Ball] = []
        self.create_balls()

        # Create IDs for drawing
        self.heptagon_id = None
        self.ball_ids = []

        # Animation timing
        self.last_time = None
        self.fps = 60
        self.update_interval = int(1000 / self.fps)

        # Start animation
        self.animate()

    def create_balls(self):
        """
        Create 20 balls at the center of the heptagon with assigned colors and numbers.
        """
        cx, cy = self.heptagon_center
        for i in range(self.num_balls):
            # Initially place them at the center with small random velocity
            ball = Ball(
                x=cx,
                y=cy,
                vx=0.0,
                vy=0.0,
                spin=0.0,
                radius=self.ball_radius,
                color=self.colors[i],
                number=i + 1,
            )
            self.balls.append(ball)

    def animate(self):
        """
        Main loop: updates positions, handles collisions, draws everything.
        """
        current_time = self.master.winfo_fpixels(
            "0i"
        )  # Use some pseudo-time measurement
        if self.last_time is None:
            self.last_time = current_time
            dt = 1.0 / self.fps
        else:
            dt = (current_time - self.last_time) / 1000.0
            if dt <= 0:
                dt = 1.0 / self.fps
            self.last_time = current_time

        # Update rotation angle
        self.rotation_angle = (self.rotation_angle + self.rotation_speed * dt) % 360

        # Update ball motion
        self.update_balls(dt)

        # Render
        self.draw_scene()

        # Schedule next frame
        self.master.after(self.update_interval, self.animate)

    def update_balls(self, dt: float):
        """
        Update the position, velocity, spin of each ball.
        Handle collisions with the rotating heptagon boundary and between balls.
        """
        # Gravity and friction
        for ball in self.balls:
            ball.vy += self.gravity
            ball.vx *= self.friction
            ball.vy *= self.friction
            ball.spin *= self.spin_friction

            ball.x += ball.vx
            ball.y += ball.vy

        # Ball-ball collisions
        self.handle_ball_collisions()

        # Ball-boundary collisions
        self.handle_heptagon_collisions()

    def handle_ball_collisions(self):
        """
        Detect and respond to collisions between every pair of balls (naive O(n^2) check).
        """
        n = len(self.balls)
        for i in range(n):
            for j in range(i + 1, n):
                b1 = self.balls[i]
                b2 = self.balls[j]
                dx = b2.x - b1.x
                dy = b2.y - b1.y
                dist_sqr = dx * dx + dy * dy
                min_dist = b1.radius + b2.radius

                if dist_sqr < min_dist * min_dist:
                    dist = math.sqrt(dist_sqr)
                    if dist < 1e-9:
                        # Avoid division by zero if balls overlap heavily
                        dist = min_dist
                        dx, dy = min_dist, 0

                    # Normalize
                    nx = dx / dist
                    ny = dy / dist

                    # Penetration overlap
                    overlap = 0.5 * (dist - min_dist)

                    # Separate
                    b1.x -= overlap * nx
                    b1.y -= overlap * ny
                    b2.x += overlap * nx
                    b2.y += overlap * ny

                    # Compute relative velocity in normal direction
                    rvx = b2.vx - b1.vx
                    rvy = b2.vy - b1.vy
                    vel_along_normal = rvx * nx + rvy * ny

                    if vel_along_normal > 0:
                        # They are separating
                        continue

                    # Bounce
                    restitution = self.restitution
                    impulse = -(1 + restitution) * vel_along_normal / 2
                    ix = impulse * nx
                    iy = impulse * ny

                    b1.vx -= ix
                    b1.vy -= iy
                    b2.vx += ix
                    b2.vy += iy

                    # Simple spin effect (optional)
                    spin_factor = 0.05
                    b1.spin += spin_factor * (rvx + rvy)
                    b2.spin -= spin_factor * (rvx + rvy)

    def handle_heptagon_collisions(self):
        """
        Handle ball collisions with the seven rotating edges.
        Each edge is rotated by self.rotation_angle around the center.
        """
        # Compute heptagon vertices in global coordinates
        cx, cy = self.heptagon_center
        angle_step = 2.0 * math.pi / self.sides
        angle_offset = math.radians(self.rotation_angle)

        # Build list of vertices
        vertices = []
        for i in range(self.sides):
            theta = angle_offset + i * angle_step
            vx = cx + self.heptagon_radius * math.cos(theta)
            vy = cy + self.heptagon_radius * math.sin(theta)
            vertices.append((vx, vy))

        # Edges: (v[i], v[i+1]) plus wrap-around
        for ball in self.balls:
            for i in range(self.sides):
                x1, y1 = vertices[i]
                x2, y2 = vertices[(i + 1) % self.sides]

                # Edge vector
                ex = x2 - x1
                ey = y2 - y1

                # Ball to vertex[0]
                bx = ball.x - x1
                by = ball.y - y1

                # Project (bx, by) onto edge vector to find the closest point on the line segment
                edge_len_sqr = ex * ex + ey * ey
                if edge_len_sqr < 1e-9:
                    continue

                t = (bx * ex + by * ey) / edge_len_sqr
                if t < 0.0:
                    t = 0.0
                elif t > 1.0:
                    t = 1.0

                # Closest point on the current edge to the ball center
                closest_x = x1 + t * ex
                closest_y = y1 + t * ey

                # Distance from ball center to this closest point
                dx = ball.x - closest_x
                dy = ball.y - closest_y
                dist_sqr = dx * dx + dy * dy
                if dist_sqr < ball.radius * ball.radius:
                    dist = math.sqrt(dist_sqr)
                    if dist < 1e-9:
                        dist = ball.radius

                    # Normal direction from edge to ball
                    nx = dx / dist
                    ny = dy / dist

                    # Penetration
                    penetration = ball.radius - dist

                    # Push ball out
                    ball.x += nx * penetration
                    ball.y += ny * penetration

                    # Reflect velocity
                    vel_along_normal = ball.vx * nx + ball.vy * ny
                    if vel_along_normal < 0:
                        ball.vx -= (1 + self.restitution) * vel_along_normal * nx
                        ball.vy -= (1 + self.restitution) * vel_along_normal * ny

                        # Spin effect (rotate number)
                        ball.spin += 0.1 * vel_along_normal

    def draw_scene(self):
        """
        Clear the canvas, draw the rotating heptagon and the balls (with their numbers).
        """
        self.canvas.delete("all")

        # Draw heptagon
        cx, cy = self.heptagon_center
        angle_offset = math.radians(self.rotation_angle)
        angle_step = 2.0 * math.pi / self.sides

        points = []
        for i in range(self.sides):
            theta = angle_offset + i * angle_step
            vx = cx + self.heptagon_radius * math.cos(theta)
            vy = cy + self.heptagon_radius * math.sin(theta)
            points.append(vx)
            points.append(vy)

        self.heptagon_id = self.canvas.create_polygon(
            *points, fill="", outline="black", width=2
        )

        # Draw balls
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius

            # Draw the ball circle
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")

            # Draw the ball number with a rotation to illustrate spin
            # We'll just offset the text by the spin mod 360 for demonstration
            text_angle = ball.spin % 360
            # To visualize spin, we can place text slightly above center
            # Or just place it center. Here we keep it center for simplicity.
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                fill="black",
                font=("Helvetica", 10, "bold"),
            )


def main():
    root = tk.Tk()
    app = HeptagonBouncingBallsApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
