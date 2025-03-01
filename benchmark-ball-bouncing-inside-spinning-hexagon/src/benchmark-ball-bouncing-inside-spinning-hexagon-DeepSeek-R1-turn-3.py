import math
import tkinter as tk
from typing import List, Tuple


class Vector2D:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float):
        return Vector2D(self.x * scalar, self.y * scalar)

    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y

    def magnitude(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self):
        mag = self.magnitude()
        return Vector2D(self.x / mag, self.y / mag) if mag != 0 else Vector2D(0, 0)


class Ball:
    def __init__(self, ball_id: int, radius: float, color: str):
        self.id = ball_id
        self.radius = radius
        self.color = color
        self.position = Vector2D(0, 0)
        self.velocity = Vector2D(0, 0)
        self.angular_velocity = 0.0
        self.angle = 0.0


class HeptagonBounce:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=800, height=800)
        self.canvas.pack()

        # Simulation parameters
        self.dt = 1 / 60
        self.gravity = Vector2D(0, 980)
        self.friction_coeff = 0.99
        self.elasticity = 0.8
        self.heptagon_radius = 350
        self.ball_radius = 15
        self.heptagon_angle = 0.0
        self.rotation_speed = 2 * math.pi / 5  # rad/s

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

        self.balls = [Ball(i + 1, self.ball_radius, self.colors[i]) for i in range(20)]

        self.init_balls()
        self.root.after(16, self.update)
        self.root.mainloop()

    def init_balls(self):
        for ball in self.balls:
            angle = 2 * math.pi * ball.id / 20
            ball.velocity = Vector2D(50 * math.cos(angle), 50 * math.sin(angle))

    def get_heptagon_points(self) -> List[Tuple[float, float]]:
        points = []
        center_x, center_y = 400, 400
        for i in range(7):
            angle = self.heptagon_angle + 2 * math.pi * i / 7
            x = center_x + self.heptagon_radius * math.cos(angle)
            y = center_y + self.heptagon_radius * math.sin(angle)
            points.append((x, y))
        return points

    def handle_collisions(self):
        # Ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1 = self.balls[i]
                b2 = self.balls[j]
                delta = b1.position - b2.position
                distance = delta.magnitude()

                if distance < 2 * self.ball_radius:
                    direction = delta.normalized()
                    overlap = (2 * self.ball_radius - distance) / 2

                    # Position correction
                    b1.position += direction * overlap
                    b2.position -= direction * overlap

                    # Velocity exchange
                    vel_along = (b1.velocity - b2.velocity).dot(direction)
                    if vel_along > 0:
                        continue

                    impulse = (1 + self.elasticity) * vel_along
                    b1.velocity -= direction * impulse
                    b2.velocity += direction * impulse

        # Ball-wall collisions
        heptagon_points = self.get_heptagon_points()
        for ball in self.balls:
            for i in range(7):
                p1 = heptagon_points[i]
                p2 = heptagon_points[(i + 1) % 7]

                # Edge normal calculation
                edge = Vector2D(p2[0] - p1[0], p2[1] - p1[1])
                normal = Vector2D(-edge.y, edge.x).normalized()

                # Project ball position onto edge
                ball_pos = Vector2D(ball.position.x, ball.position.y)
                closest_point = self.project_point_on_segment(
                    ball_pos, Vector2D(p1[0], p1[1]), Vector2D(p2[0], p2[1])
                )

                dist_vector = ball_pos - closest_point
                distance = dist_vector.magnitude()

                if distance < self.ball_radius:
                    penetration = self.ball_radius - distance
                    ball.position += dist_vector.normalized() * penetration

                    vel_normal = ball.velocity.dot(normal)
                    if vel_normal < 0:
                        ball.velocity -= normal * (1 + self.elasticity) * vel_normal

                    # Apply friction
                    tangent = Vector2D(normal.y, -normal.x)
                    vel_tangent = ball.velocity.dot(tangent)
                    ball.velocity -= tangent * vel_tangent * 0.2

    def project_point_on_segment(
        self, point: Vector2D, seg_a: Vector2D, seg_b: Vector2D
    ) -> Vector2D:
        ap = point - seg_a
        ab = seg_b - seg_a
        t = ap.dot(ab) / ab.dot(ab)
        t = max(0, min(1, t))
        return seg_a + ab * t

    def update_physics(self):
        self.heptagon_angle += self.rotation_speed * self.dt
        self.heptagon_angle %= 2 * math.pi

        for ball in self.balls:
            # Apply gravity and friction
            ball.velocity += self.gravity * self.dt
            ball.velocity *= self.friction_coeff**self.dt

            # Update position
            ball.position += ball.velocity * self.dt

            # Update rotation
            ball.angular_velocity *= 0.99**self.dt
            ball.angle += ball.angular_velocity * self.dt

    def draw(self):
        self.canvas.delete("all")

        # Draw heptagon
        points = self.get_heptagon_points()
        self.canvas.create_polygon(
            [coord for point in points for coord in point],
            outline="black",
            fill="",
            width=2,
        )

        # Draw balls
        for ball in self.balls:
            x, y = ball.position.x + 400, ball.position.y + 400
            self.canvas.create_oval(
                x - ball.radius,
                y - ball.radius,
                x + ball.radius,
                y + ball.radius,
                fill=ball.color,
                outline="black",
            )

            # Draw number with rotation
            text_angle = math.degrees(ball.angle) % 360
            self.canvas.create_text(
                x, y, text=str(ball.id), angle=text_angle, font=("Arial", 10, "bold")
            )

    def update(self):
        self.update_physics()
        self.handle_collisions()
        self.draw()
        self.root.after(16, self.update)


if __name__ == "__main__":
    HeptagonBounce()
