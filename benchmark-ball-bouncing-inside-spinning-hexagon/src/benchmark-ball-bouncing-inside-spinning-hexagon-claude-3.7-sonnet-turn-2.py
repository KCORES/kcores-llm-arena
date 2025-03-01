import tkinter as tk
import math
import numpy as np
import random
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
GRAVITY = 9.8 * 100  # Scaled gravity (pixels/s^2)
FRICTION = 0.98  # Friction coefficient
ANGULAR_FRICTION = 0.98  # Angular friction coefficient
RESTITUTION = 0.8  # Coefficient of restitution (bounciness)
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Distance from center to vertices
HEPTAGON_ROTATION_SPEED = (
    2 * math.pi / 5
)  # 360 degrees per 5 seconds (in radians/second)
FPS = 60
TIME_STEP = 1 / FPS

# Ball colors
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0  # Rotation angle
    angular_velocity: float = 0  # Angular velocity


@dataclass
class Line:
    x1: float
    y1: float
    x2: float
    y2: float

    def normal(self) -> Tuple[float, float]:
        """Return the normal vector to this line (perpendicular)"""
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0:
            return (0, 0)
        # Perpendicular vector (rotated 90 degrees)
        return (-dy / length, dx / length)


class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")

        self.canvas = tk.Canvas(
            root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="black"
        )
        self.canvas.pack()

        # Initialize heptagon vertices
        self.heptagon_angle = 0
        self.heptagon_vertices = self.calculate_heptagon_vertices()

        # Initialize balls
        self.balls = []
        for i in range(20):
            # All balls start at the center with random small velocities
            ball = Ball(
                x=WINDOW_WIDTH / 2,
                y=WINDOW_HEIGHT / 2,
                vx=random.uniform(-50, 50),
                vy=random.uniform(-50, 50),
                radius=BALL_RADIUS,
                color=COLORS[i],
                number=i + 1,
            )
            self.balls.append(ball)

        # Start animation
        self.update()

    def calculate_heptagon_vertices(self) -> List[Tuple[float, float]]:
        """Calculate the vertices of the heptagon based on current rotation"""
        vertices = []
        center_x, center_y = WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2

        for i in range(7):
            angle = self.heptagon_angle + i * (2 * math.pi / 7)
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))

        return vertices

    def get_heptagon_lines(self) -> List[Line]:
        """Get the lines that make up the heptagon"""
        lines = []
        for i in range(7):
            x1, y1 = self.heptagon_vertices[i]
            x2, y2 = self.heptagon_vertices[(i + 1) % 7]
            lines.append(Line(x1, y1, x2, y2))
        return lines

    def draw_heptagon(self):
        """Draw the heptagon on the canvas"""
        points = []
        for x, y in self.heptagon_vertices:
            points.extend([x, y])

        self.canvas.create_polygon(points, outline="white", fill="", width=2)

    def draw_balls(self):
        """Draw all balls on the canvas"""
        for ball in self.balls:
            # Draw the ball
            self.canvas.create_oval(
                ball.x - ball.radius,
                ball.y - ball.radius,
                ball.x + ball.radius,
                ball.y + ball.radius,
                fill=ball.color,
                outline="white",
            )

            # Draw the number (rotated to show spin)
            text_x = ball.x + 0.5 * ball.radius * math.sin(ball.angle)
            text_y = ball.y - 0.5 * ball.radius * math.cos(ball.angle)

            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(ball.radius * 0.8)),
            )

    def distance_point_to_line(self, px: float, py: float, line: Line) -> float:
        """Calculate the distance from a point to a line segment"""
        x1, y1, x2, y2 = line.x1, line.y1, line.x2, line.y2

        # Vector from line start to point
        vx = px - x1
        vy = py - y1

        # Vector representing the line
        ux = x2 - x1
        uy = y2 - y1

        # Length of the line
        length = math.sqrt(ux * ux + uy * uy)
        if length == 0:
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # Normalize the line vector
        ux /= length
        uy /= length

        # Project point onto line
        projection = vx * ux + vy * uy

        # Clamp projection to line segment
        projection = max(0, min(length, projection))

        # Find the closest point on the line
        closest_x = x1 + projection * ux
        closest_y = y1 + projection * uy

        # Return the distance to the closest point
        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    def check_ball_wall_collision(self, ball: Ball, line: Line) -> bool:
        """Check if a ball is colliding with a wall (line)"""
        distance = self.distance_point_to_line(ball.x, ball.y, line)
        return distance <= ball.radius

    def resolve_ball_wall_collision(self, ball: Ball, line: Line):
        """Resolve collision between a ball and a wall"""
        # Get the normal vector to the line
        nx, ny = line.normal()

        # Calculate the dot product of velocity and normal
        dot_product = ball.vx * nx + ball.vy * ny

        # Only bounce if the ball is moving toward the wall
        if dot_product < 0:
            # Calculate reflection vector
            ball.vx = ball.vx - 2 * dot_product * nx
            ball.vy = ball.vy - 2 * dot_product * ny

            # Apply restitution (energy loss)
            ball.vx *= RESTITUTION
            ball.vy *= RESTITUTION

            # Add some angular velocity based on impact
            impact = abs(dot_product)
            tangent_x, tangent_y = ny, -nx  # Tangent to the wall
            tangential_velocity = ball.vx * tangent_x + ball.vy * tangent_y
            ball.angular_velocity += tangential_velocity * 0.05

            # Move the ball outside the wall to prevent sticking
            penetration = ball.radius - self.distance_point_to_line(
                ball.x, ball.y, line
            )
            if penetration > 0:
                ball.x += nx * penetration * 1.01
                ball.y += ny * penetration * 1.01

    def check_ball_ball_collision(self, ball1: Ball, ball2: Ball) -> bool:
        """Check if two balls are colliding"""
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y
        distance = math.sqrt(dx * dx + dy * dy)
        return distance < ball1.radius + ball2.radius

    def resolve_ball_ball_collision(self, ball1: Ball, ball2: Ball):
        """Resolve collision between two balls"""
        # Vector from ball1 to ball2
        dx = ball2.x - ball1.x
        dy = ball2.y - ball1.y

        # Distance between balls
        distance = math.sqrt(dx * dx + dy * dy)

        if distance == 0:  # Avoid division by zero
            return

        # Normalize the collision vector
        nx = dx / distance
        ny = dy / distance

        # Calculate relative velocity
        dvx = ball2.vx - ball1.vx
        dvy = ball2.vy - ball1.vy

        # Calculate velocity along the normal
        normal_velocity = dvx * nx + dvy * ny

        # Only resolve if balls are moving toward each other
        if normal_velocity > 0:
            return

        # Calculate impulse
        impulse = -(1 + RESTITUTION) * normal_velocity / 2

        # Apply impulse
        ball1.vx -= impulse * nx
        ball1.vy -= impulse * ny
        ball2.vx += impulse * nx
        ball2.vy += impulse * ny

        # Update angular velocities based on tangential component
        tangent_x, tangent_y = -ny, nx  # Perpendicular to normal
        tangential_velocity = dvx * tangent_x + dvy * tangent_y

        ball1.angular_velocity -= tangential_velocity * 0.1
        ball2.angular_velocity += tangential_velocity * 0.1

        # Separate the balls to prevent sticking
        overlap = (ball1.radius + ball2.radius - distance) / 2
        ball1.x -= nx * overlap * 1.01
        ball1.y -= ny * overlap * 1.01
        ball2.x += nx * overlap * 1.01
        ball2.y += ny * overlap * 1.01

    def update(self):
        """Update the simulation by one time step"""
        # Update heptagon rotation
        self.heptagon_angle += HEPTAGON_ROTATION_SPEED * TIME_STEP
        self.heptagon_vertices = self.calculate_heptagon_vertices()
        heptagon_lines = self.get_heptagon_lines()

        # Update ball positions and velocities
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY * TIME_STEP

            # Update position
            ball.x += ball.vx * TIME_STEP
            ball.y += ball.vy * TIME_STEP

            # Update rotation
            ball.angle += ball.angular_velocity * TIME_STEP

            # Apply friction
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.angular_velocity *= ANGULAR_FRICTION

            # Check for wall collisions
            for line in heptagon_lines:
                if self.check_ball_wall_collision(ball, line):
                    self.resolve_ball_wall_collision(ball, line)

        # Check for ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                if self.check_ball_ball_collision(self.balls[i], self.balls[j]):
                    self.resolve_ball_ball_collision(self.balls[i], self.balls[j])

        # Clear canvas and redraw everything
        self.canvas.delete("all")
        self.draw_heptagon()
        self.draw_balls()

        # Schedule the next update
        self.root.after(int(TIME_STEP * 1000), self.update)


def main():
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
