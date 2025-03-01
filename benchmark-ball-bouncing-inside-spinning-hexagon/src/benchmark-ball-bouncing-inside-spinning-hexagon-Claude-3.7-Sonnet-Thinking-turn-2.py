import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import sys

# Physics constants
GRAVITY = 0.5
FRICTION = 0.98
BOUNCE_DAMPING = 0.8
ROTATION_FRICTION = 0.95

# Window dimensions
WIDTH = 800
HEIGHT = 800

# Heptagon and ball properties
HEPTAGON_RADIUS = 300
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angle: float = 0  # For rotation
    angular_velocity: float = 0

    def update(self):
        # Apply gravity
        self.vy += GRAVITY

        # Apply velocities
        self.x += self.vx
        self.y += self.vy

        # Apply friction
        self.vx *= FRICTION
        self.vy *= FRICTION

        # Update rotation
        self.angle += self.angular_velocity
        self.angular_velocity *= ROTATION_FRICTION


class Simulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="black")
        self.canvas.pack()

        # Initialize the heptagon vertices
        self.heptagon_angle = 0
        self.heptagon_angular_velocity = (
            2 * math.pi / (5 * 60)
        )  # 360 degrees per 5 seconds
        self.heptagon_vertices = self.calculate_heptagon_vertices()

        # Create the balls
        self.balls = []
        for i in range(NUM_BALLS):
            self.balls.append(
                Ball(
                    x=WIDTH // 2,
                    y=HEIGHT // 2,
                    vx=(np.random.random() - 0.5) * 3,  # Random initial velocity
                    vy=(np.random.random() - 0.5) * 3,
                    radius=BALL_RADIUS,
                    color=BALL_COLORS[i],
                    number=i + 1,
                    angular_velocity=(np.random.random() - 0.5) * 0.2,
                )
            )

        # Start the animation
        self.update()

    def calculate_heptagon_vertices(self):
        vertices = []
        center_x, center_y = WIDTH // 2, HEIGHT // 2

        for i in range(7):
            angle = self.heptagon_angle + i * (2 * math.pi / 7)
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))

        return vertices

    def draw_heptagon(self):
        points = []
        for vertex in self.heptagon_vertices:
            points.extend(vertex)
        self.canvas.create_polygon(points, outline="white", fill="", width=2)

    def draw_balls(self):
        for ball in self.balls:
            # Draw the ball
            x, y = ball.x, ball.y
            self.canvas.create_oval(
                x - ball.radius,
                y - ball.radius,
                x + ball.radius,
                y + ball.radius,
                fill=ball.color,
                outline="white",
            )

            # Draw the number (indicating rotation)
            text_x = x + 0.7 * ball.radius * math.cos(ball.angle)
            text_y = y + 0.7 * ball.radius * math.sin(ball.angle)
            self.canvas.create_text(
                text_x,
                text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", 10, "bold"),
            )

    def check_wall_collision(self, ball):
        # For each edge of the heptagon
        for i in range(7):
            p1 = self.heptagon_vertices[i]
            p2 = self.heptagon_vertices[(i + 1) % 7]

            # Check collision with this edge
            collision, normal = self.line_circle_collision(
                p1, p2, (ball.x, ball.y), ball.radius
            )

            if collision:
                # Calculate the reflection vector
                dot_product = ball.vx * normal[0] + ball.vy * normal[1]
                ball.vx -= 2 * dot_product * normal[0]
                ball.vy -= 2 * dot_product * normal[1]

                # Apply bounce damping
                ball.vx *= BOUNCE_DAMPING
                ball.vy *= BOUNCE_DAMPING

                # Add spin based on impact
                tangent = (-normal[1], normal[0])
                tangential_velocity = ball.vx * tangent[0] + ball.vy * tangent[1]
                ball.angular_velocity += tangential_velocity * 0.02

                # Adjust position to avoid sticking to the wall
                overlap = ball.radius - self.point_line_distance(
                    (ball.x, ball.y), p1, p2
                )
                ball.x += normal[0] * overlap * 1.01  # Move slightly away from wall
                ball.y += normal[1] * overlap * 1.01

                return True
        return False

    def check_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]

                # Calculate distance between balls
                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                distance = math.sqrt(dx * dx + dy * dy)

                # Check if they're colliding
                if distance < ball1.radius + ball2.radius:
                    # Calculate normal and tangent vectors
                    if distance == 0:  # Avoid division by zero
                        nx, ny = 1, 0
                    else:
                        nx = dx / distance
                        ny = dy / distance

                    # Calculate relative velocity
                    dvx = ball2.vx - ball1.vx
                    dvy = ball2.vy - ball1.vy

                    # Calculate the dot product (how much velocity is along the normal)
                    dot_product = dvx * nx + dvy * ny

                    # Only bounce if they're moving toward each other
                    if dot_product < 0:
                        # Calculate impulse
                        impulse = -2 * dot_product

                        # Update velocities
                        ball1.vx -= impulse * nx / 2
                        ball1.vy -= impulse * ny / 2
                        ball2.vx += impulse * nx / 2
                        ball2.vy += impulse * ny / 2

                        # Update rotations based on impact
                        tangent = (-ny, nx)
                        tangential_velocity1 = (
                            ball1.vx * tangent[0] + ball1.vy * tangent[1]
                        )
                        tangential_velocity2 = (
                            ball2.vx * tangent[0] + ball2.vy * tangent[1]
                        )
                        ball1.angular_velocity += (
                            tangential_velocity2 - tangential_velocity1
                        ) * 0.02
                        ball2.angular_velocity += (
                            tangential_velocity1 - tangential_velocity2
                        ) * 0.02

                        # Separate the balls to avoid sticking
                        overlap = (ball1.radius + ball2.radius - distance) / 2
                        ball1.x -= nx * overlap * 1.01
                        ball1.y -= ny * overlap * 1.01
                        ball2.x += nx * overlap * 1.01
                        ball2.y += ny * overlap * 1.01

    def line_circle_collision(self, p1, p2, center, radius):
        # Calculate the closest point on the line segment to the circle center
        p1_vec = np.array(p1)
        p2_vec = np.array(p2)
        center_vec = np.array(center)

        line_vec = p2_vec - p1_vec
        line_len = np.linalg.norm(line_vec)

        if line_len == 0:  # Avoid division by zero
            return False, None

        line_unitvec = line_vec / line_len

        # Vector from p1 to circle center
        p1_to_center = center_vec - p1_vec

        # Project p1_to_center onto the line vector
        projection_length = np.dot(p1_to_center, line_unitvec)

        # Clamp projection to line segment
        projection_length = max(0, min(line_len, projection_length))

        # Calculate the closest point on the line
        closest_point = p1_vec + projection_length * line_unitvec

        # Get the distance from the closest point to the circle center
        dist_vec = center_vec - closest_point
        distance = np.linalg.norm(dist_vec)

        # Check if there's a collision
        if distance <= radius:
            # Normalize the collision normal
            if distance > 0:
                normal = dist_vec / distance
            else:
                # If the circle center is exactly on the line, use a perpendicular vector
                normal = np.array([-line_unitvec[1], line_unitvec[0]])

            return True, normal

        return False, None

    def point_line_distance(self, point, line_p1, line_p2):
        # Calculate distance from point to line defined by line_p1 and line_p2
        p1_vec = np.array(line_p1)
        p2_vec = np.array(line_p2)
        point_vec = np.array(point)

        line_vec = p2_vec - p1_vec
        line_len = np.linalg.norm(line_vec)

        if line_len == 0:  # Avoid division by zero
            return np.linalg.norm(point_vec - p1_vec)

        line_unitvec = line_vec / line_len

        # Vector from p1 to point
        p1_to_point = point_vec - p1_vec

        # Project p1_to_point onto the line vector
        projection_length = np.dot(p1_to_point, line_unitvec)
        projection_length = max(0, min(line_len, projection_length))

        # Calculate the closest point on the line
        closest_point = p1_vec + projection_length * line_unitvec

        # Distance between point and closest point
        return np.linalg.norm(point_vec - closest_point)

    def update(self):
        # Clear the canvas
        self.canvas.delete("all")

        # Update heptagon rotation (360 degrees per 5 seconds)
        self.heptagon_angle += self.heptagon_angular_velocity
        self.heptagon_vertices = self.calculate_heptagon_vertices()

        # Draw the heptagon
        self.draw_heptagon()

        # Update and draw the balls
        for ball in self.balls:
            ball.update()
            self.check_wall_collision(ball)

        # Check collisions between balls
        self.check_ball_collisions()

        # Draw the balls
        self.draw_balls()

        # Schedule the next update
        self.root.after(16, self.update)  # Approximately 60 FPS


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    sim = Simulation(root)
    root.mainloop()
