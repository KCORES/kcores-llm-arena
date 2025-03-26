import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
GRAVITY = 0.2
FRICTION = 0.99
BALL_FRICTION = 0.995
ROTATIONAL_FRICTION = 0.98
BOUNCE_LOSS = 0.8
HEPTAGON_SPEED = 360 / (5 * 60)  # degrees per frame (60 FPS)
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300

# Colors for the balls
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angular_velocity: float = 0.0
    angle: float = 0.0


@dataclass
class Heptagon:
    radius: float
    rotation: float = 0.0
    points: List[Tuple[float, float]] = None

    def __post_init__(self):
        self.update_points()

    def update_points(self):
        self.points = []
        for i in range(7):
            angle = math.radians(self.rotation + i * 360 / 7)
            x = CENTER_X + self.radius * math.cos(angle)
            y = CENTER_Y + self.radius * math.sin(angle)
            self.points.append((x, y))

    def rotate(self, degrees):
        self.rotation += degrees
        self.update_points()


class BouncingBallsSimulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.heptagon = Heptagon(HEPTAGON_RADIUS)
        self.balls = self.create_balls()
        self.running = True

        self.setup_bindings()
        self.animate()

    def create_balls(self) -> List[Ball]:
        balls = []
        for i in range(20):
            # Start all balls at center with small random velocity
            angle = np.random.uniform(0, 2 * math.pi)
            speed = np.random.uniform(0.5, 2)
            vx = speed * math.cos(angle)
            vy = speed * math.sin(angle)

            balls.append(
                Ball(
                    x=CENTER_X,
                    y=CENTER_Y,
                    vx=vx,
                    vy=vy,
                    radius=BALL_RADIUS,
                    color=BALL_COLORS[i],
                    number=i + 1,
                    angular_velocity=np.random.uniform(-2, 2),
                )
            )
        return balls

    def setup_bindings(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.canvas.bind("<Button-1>", self.on_click)

    def on_close(self):
        self.running = False
        self.root.destroy()

    def on_click(self, event):
        # Reset simulation on click
        self.balls = self.create_balls()

    def animate(self):
        if not self.running:
            return

        self.update_physics()
        self.draw()
        self.root.after(16, self.animate)  # ~60 FPS

    def update_physics(self):
        # Rotate heptagon
        self.heptagon.rotate(HEPTAGON_SPEED)

        # Apply gravity and friction to balls
        for ball in self.balls:
            ball.vy += GRAVITY
            ball.vx *= FRICTION
            ball.vy *= FRICTION

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Update rotation
            ball.angle += ball.angular_velocity
            ball.angular_velocity *= ROTATIONAL_FRICTION

        # Handle ball-wall collisions
        self.handle_wall_collisions()

        # Handle ball-ball collisions
        self.handle_ball_collisions()

    def handle_wall_collisions(self):
        for ball in self.balls:
            # Check collision with heptagon walls
            for i in range(7):
                p1 = self.heptagon.points[i]
                p2 = self.heptagon.points[(i + 1) % 7]

                # Find closest point on segment to ball
                closest = self.closest_point_on_segment(p1, p2, (ball.x, ball.y))
                distance = math.hypot(ball.x - closest[0], ball.y - closest[1])

                if distance < ball.radius:
                    # Calculate normal vector
                    nx = (ball.x - closest[0]) / distance
                    ny = (ball.y - closest[1]) / distance

                    # Calculate relative velocity along normal
                    rel_vel = ball.vx * nx + ball.vy * ny

                    # Only collide if moving towards the wall
                    if rel_vel < 0:
                        # Calculate impulse
                        j = -(1 + BOUNCE_LOSS) * rel_vel

                        # Apply impulse
                        ball.vx += j * nx
                        ball.vy += j * ny

                        # Add some angular velocity from friction with wall
                        tangent_vel = -ball.vx * ny + ball.vy * nx
                        ball.angular_velocity += tangent_vel * 0.01

                        # Move ball out of collision
                        overlap = ball.radius - distance
                        ball.x += overlap * nx
                        ball.y += overlap * ny

    def handle_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.hypot(dx, dy)

                if distance < ball1.radius + ball2.radius:
                    # Normalize the distance vector
                    if distance > 0:
                        nx = dx / distance
                        ny = dy / distance
                    else:
                        nx, ny = 1, 0

                    # Calculate relative velocity
                    rel_vel_x = ball2.vx - ball1.vx
                    rel_vel_y = ball2.vy - ball1.vy

                    # Calculate relative velocity in terms of the normal direction
                    vel_along_normal = rel_vel_x * nx + rel_vel_y * ny

                    # Do not resolve if velocities are separating
                    if vel_along_normal > 0:
                        continue

                    # Calculate restitution
                    e = BOUNCE_LOSS

                    # Calculate impulse scalar
                    j = -(1 + e) * vel_along_normal
                    j /= 1 / ball1.radius + 1 / ball2.radius

                    # Apply impulse
                    impulse_x = j * nx
                    impulse_y = j * ny

                    ball1.vx -= impulse_x / ball1.radius
                    ball1.vy -= impulse_y / ball1.radius
                    ball2.vx += impulse_x / ball2.radius
                    ball2.vy += impulse_y / ball2.radius

                    # Add some angular velocity from friction
                    tangent_vel = -rel_vel_x * ny + rel_vel_y * nx
                    ball1.angular_velocity -= tangent_vel * 0.005
                    ball2.angular_velocity += tangent_vel * 0.005

                    # Move balls apart
                    overlap = (ball1.radius + ball2.radius - distance) / 2
                    ball1.x -= overlap * nx
                    ball1.y -= overlap * ny
                    ball2.x += overlap * nx
                    ball2.y += overlap * ny

    def closest_point_on_segment(self, p1, p2, p):
        x1, y1 = p1
        x2, y2 = p2
        px, py = p

        # Vector from p1 to p2
        dx = x2 - x1
        dy = y2 - y1

        # Vector from p1 to p
        dpx = px - x1
        dpy = py - y1

        # Projection of p onto the segment
        dot = dpx * dx + dpy * dy
        len_sq = dx * dx + dy * dy

        if len_sq == 0:
            return p1

        t = max(0, min(1, dot / len_sq))

        return (x1 + t * dx, y1 + t * dy)

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon
        self.canvas.create_polygon(
            self.heptagon.points, outline="black", fill="", width=2
        )

        # Draw balls
        for ball in self.balls:
            # Draw ball
            self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="black",
            )

            # Draw number with rotation
            text_x = ball.x + math.cos(ball.angle) * ball.radius * 0.6
            text_y = ball.y + math.sin(ball.angle) * ball.radius * 0.6
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="black",
                font=("Arial", int(ball.radius * 0.8)),
            )


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    simulation = BouncingBallsSimulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
