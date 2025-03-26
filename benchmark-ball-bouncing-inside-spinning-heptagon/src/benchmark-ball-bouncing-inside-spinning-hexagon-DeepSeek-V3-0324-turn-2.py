import tkinter as tk
import math
import numpy as np
import random
from typing import List, Tuple, Optional

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 15
GRAVITY = 0.2
FRICTION = 0.99
RESTITUTION = 0.8
ROTATION_SPEED = 360 / 5  # degrees per second
SPIN_FRICTION = 0.98

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


class Ball:
    def __init__(self, x: float, y: float, radius: float, number: int, color: str):
        self.x = x
        self.y = y
        self.radius = radius
        self.number = number
        self.color = color
        self.vx = 0
        self.vy = 0
        self.spin = random.uniform(-5, 5)
        self.rotation = 0
        self.mass = radius**2  # Mass proportional to area

    def update(self):
        # Apply gravity
        self.vy += GRAVITY

        # Apply friction
        self.vx *= FRICTION
        self.vy *= FRICTION

        # Update position
        self.x += self.vx
        self.y += self.vy

        # Update rotation
        self.spin *= SPIN_FRICTION
        self.rotation += self.spin

    def draw(
        self, canvas: tk.Canvas, offset_x: float, offset_y: float, rotation: float
    ):
        # Calculate screen position
        screen_x = self.x + offset_x
        screen_y = self.y + offset_y

        # Draw the ball
        canvas.create_oval(
            screen_x - self.radius,
            screen_y - self.radius,
            screen_x + self.radius,
            screen_y + self.radius,
            fill=self.color,
            outline="black",
        )

        # Draw the number with rotation
        angle_rad = math.radians(self.rotation)
        text_x = screen_x + math.cos(angle_rad) * self.radius * 0.6
        text_y = screen_y + math.sin(angle_rad) * self.radius * 0.6
        canvas.create_text(text_x, text_y, text=str(self.number), fill="black")


class Heptagon:
    def __init__(self, center_x: float, center_y: float, radius: float):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.rotation = 0
        self.points = self.calculate_points()

    def calculate_points(self) -> List[Tuple[float, float]]:
        points = []
        for i in range(7):
            angle = math.radians(self.rotation + i * 360 / 7)
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            points.append((x, y))
        return points

    def update(self, dt: float):
        self.rotation += ROTATION_SPEED * dt
        self.points = self.calculate_points()

    def draw(self, canvas: tk.Canvas):
        # Draw the heptagon
        canvas.create_polygon(
            *[coord for point in self.points for coord in point],
            fill="",
            outline="black",
            width=2
        )

        # Draw center marker
        canvas.create_oval(
            self.center_x - 5,
            self.center_y - 5,
            self.center_x + 5,
            self.center_y + 5,
            fill="red",
        )


class PhysicsEngine:
    @staticmethod
    def check_ball_collision(ball1: Ball, ball2: Ball) -> bool:
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance < (ball1.radius + ball2.radius)

    @staticmethod
    def resolve_ball_collision(ball1: Ball, ball2: Ball):
        # Calculate collision normal
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Normalize
        nx = dx / distance
        ny = dy / distance

        # Calculate relative velocity
        vx = ball2.vx - ball1.vx
        vy = ball2.vy - ball1.vy

        # Calculate relative velocity in terms of the normal direction
        vel_along_normal = vx * nx + vy * ny

        # Do not resolve if velocities are separating
        if vel_along_normal > 0:
            return

        # Calculate impulse scalar
        restitution = RESTITUTION
        j = -(1 + restitution) * vel_along_normal
        j /= 1 / ball1.mass + 1 / ball2.mass

        # Apply impulse
        impulse_x = j * nx
        impulse_y = j * ny

        ball1.vx -= impulse_x / ball1.mass
        ball1.vy -= impulse_y / ball1.mass
        ball2.vx += impulse_x / ball2.mass
        ball2.vy += impulse_y / ball2.mass

        # Add some spin based on collision
        ball1.spin += (impulse_x - impulse_y) * 0.1
        ball2.spin += (impulse_x - impulse_y) * 0.1

        # Separate balls to prevent sticking
        overlap = (ball1.radius + ball2.radius - distance) / 2.0
        ball1.x -= overlap * nx
        ball1.y -= overlap * ny
        ball2.x += overlap * nx
        ball2.y += overlap * ny

    @staticmethod
    def check_heptagon_collision(
        ball: Ball, heptagon: Heptagon
    ) -> Optional[Tuple[float, float]]:
        # Transform ball position to heptagon's local space (considering rotation)
        dx = ball.x - heptagon.center_x
        dy = ball.y - heptagon.center_y
        angle = -math.radians(heptagon.rotation)

        # Rotate the ball position
        local_x = dx * math.cos(angle) - dy * math.sin(angle)
        local_y = dx * math.sin(angle) + dy * math.cos(angle)

        # Check if ball is outside the heptagon
        distance = math.sqrt(local_x**2 + local_y**2)
        if distance + ball.radius <= heptagon.radius:
            return None

        # Find the closest point on the heptagon
        angle_to_ball = math.atan2(local_y, local_x)
        segment = (angle_to_ball + math.pi) % (2 * math.pi) / (2 * math.pi / 7)
        segment_idx = int(segment)

        # Get the two vertices of the segment
        angle1 = segment_idx * (2 * math.pi / 7)
        angle2 = (segment_idx + 1) * (2 * math.pi / 7)

        x1, y1 = math.cos(angle1) * heptagon.radius, math.sin(angle1) * heptagon.radius
        x2, y2 = math.cos(angle2) * heptagon.radius, math.sin(angle2) * heptagon.radius

        # Find the closest point on the segment
        closest_point = PhysicsEngine.closest_point_on_segment(
            local_x, local_y, x1, y1, x2, y2
        )

        # Calculate normal
        nx = local_x - closest_point[0]
        ny = local_y - closest_point[1]
        distance_to_edge = math.sqrt(nx**2 + ny**2)

        if distance_to_edge < ball.radius:
            # Normalize normal
            nx /= distance_to_edge
            ny /= distance_to_edge

            # Rotate normal back to world space
            world_nx = nx * math.cos(-angle) - ny * math.sin(-angle)
            world_ny = nx * math.sin(-angle) + ny * math.cos(-angle)

            return (world_nx, world_ny)
        return None

    @staticmethod
    def closest_point_on_segment(
        px: float, py: float, x1: float, y1: float, x2: float, y2: float
    ) -> Tuple[float, float]:
        # Vector from p1 to p2
        dx = x2 - x1
        dy = y2 - y1

        # Vector from p1 to p
        px = px - x1
        py = py - y1

        # Project p onto the segment
        d = dx * dx + dy * dy
        if d == 0:
            return (x1, y1)

        t = max(0, min(1, (px * dx + py * dy) / d))

        return (x1 + t * dx, y1 + t * dy)

    @staticmethod
    def resolve_heptagon_collision(ball: Ball, normal: Tuple[float, float]):
        # Calculate velocity along normal
        vel_along_normal = ball.vx * normal[0] + ball.vy * normal[1]

        # Apply bounce with restitution
        ball.vx -= (1 + RESTITUTION) * vel_along_normal * normal[0]
        ball.vy -= (1 + RESTITUTION) * vel_along_normal * normal[1]

        # Add some spin based on collision
        ball.spin += (ball.vx - ball.vy) * 0.1

        # Limit bounce height
        if ball.vy < 0 and abs(ball.vy) > math.sqrt(
            2 * GRAVITY * (HEPTAGON_RADIUS - ball.radius)
        ):
            ball.vy = -math.sqrt(2 * GRAVITY * (HEPTAGON_RADIUS - ball.radius))


class BouncingBallsApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Bouncing Balls in a Spinning Heptagon")

        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.heptagon = Heptagon(WIDTH / 2, HEIGHT / 2, HEPTAGON_RADIUS)

        # Create balls
        self.balls = []
        for i in range(20):
            # Start all balls at center with small random velocity
            ball = Ball(
                x=WIDTH / 2,
                y=HEIGHT / 2,
                radius=BALL_RADIUS,
                number=i + 1,
                color=BALL_COLORS[i],
            )
            ball.vx = random.uniform(-2, 2)
            ball.vy = random.uniform(-2, 2)
            self.balls.append(ball)

        self.last_time = 0
        self.running = True
        self.root.after(10, self.update)

        # Bind close event
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_close(self):
        self.running = False
        self.root.destroy()

    def update(self):
        current_time = self.root.tk.call("after", "info")  # Approximate time tracking
        dt = 0.016  # Approximate 60 FPS

        # Update physics
        self.heptagon.update(dt)

        # Update balls
        for ball in self.balls:
            ball.update()

            # Check collision with heptagon
            normal = PhysicsEngine.check_heptagon_collision(ball, self.heptagon)
            if normal:
                PhysicsEngine.resolve_heptagon_collision(ball, normal)

        # Check ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                if PhysicsEngine.check_ball_collision(self.balls[i], self.balls[j]):
                    PhysicsEngine.resolve_ball_collision(self.balls[i], self.balls[j])

        # Draw everything
        self.canvas.delete("all")
        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas, 0, 0, 0)

        # Schedule next update
        if self.running:
            self.root.after(16, self.update)


if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
