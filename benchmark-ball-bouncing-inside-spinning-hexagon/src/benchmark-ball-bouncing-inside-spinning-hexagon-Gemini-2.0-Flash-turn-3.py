import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Ball:
    x: float
    y: float
    radius: float
    number: int
    color: str
    vx: float = 0.0
    vy: float = 0.0
    angular_velocity: float = 0.0
    rotation_angle: float = 0.0


class HeptagonBouncer:
    def __init__(self, master, width=800, height=600, num_balls=20):
        self.master = master
        master.title("Heptagon Bouncer")
        self.width = width
        self.height = height
        self.canvas = tk.Canvas(
            master, width=self.width, height=self.height, bg="white"
        )
        self.canvas.pack()

        self.num_balls = num_balls
        self.ball_radius = 20
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
        self.balls: List[Ball] = []
        self.gravity = 0.5
        self.friction = 0.01
        self.ball_friction = 0.005  # Friction to slow down the ball's spin

        self.heptagon_radius = (
            min(self.width, self.height) / 2 * 0.8
        )  # Size of heptagon
        self.heptagon_center_x = self.width / 2
        self.heptagon_center_y = self.height / 2
        self.heptagon_angle = 0.0
        self.heptagon_rotation_speed = 360 / 5  # degrees per second

        self.init_balls()
        self.draw_heptagon()
        self.update()

    def init_balls(self):
        for i in range(self.num_balls):
            self.balls.append(
                Ball(
                    x=self.heptagon_center_x,
                    y=self.heptagon_center_y,
                    radius=self.ball_radius,
                    number=i + 1,
                    color=self.colors[i % len(self.colors)],
                    vx=0.0,
                    vy=0.0,
                    angular_velocity=0.0,  # Initial spin
                )
            )

    def draw_heptagon(self):
        points = []
        for i in range(7):
            angle = math.radians(self.heptagon_angle + i * 360 / 7)
            x = self.heptagon_center_x + self.heptagon_radius * math.cos(angle)
            y = self.heptagon_center_y + self.heptagon_radius * math.sin(angle)
            points.append(x)
            points.append(y)
        self.canvas.create_polygon(points, outline="black", fill="")
        self.heptagon_points = [
            (points[i], points[i + 1]) for i in range(0, len(points), 2)
        ]

    def update(self):
        self.move_balls()
        self.handle_collisions()
        self.rotate_heptagon()
        self.draw()
        self.master.after(10, self.update)  # milliseconds

    def move_balls(self):
        for ball in self.balls:
            ball.vy += self.gravity
            ball.x += ball.vx
            ball.y += ball.vy
            ball.rotation_angle += ball.angular_velocity

            # Apply friction to linear motion
            ball.vx *= 1 - self.friction
            ball.vy *= 1 - self.friction

            # Apply friction to angular motion
            if ball.angular_velocity > 0:
                ball.angular_velocity = max(
                    0, ball.angular_velocity - self.ball_friction
                )
            else:
                ball.angular_velocity = min(
                    0, ball.angular_velocity + self.ball_friction
                )

    def handle_collisions(self):
        self.handle_wall_collisions()
        self.handle_ball_collisions()

    def handle_wall_collisions(self):
        for ball in self.balls:
            for i in range(7):
                x1, y1 = self.heptagon_points[i]
                x2, y2 = self.heptagon_points[(i + 1) % 7]

                distance = self.distance_to_line(ball.x, ball.y, x1, y1, x2, y2)

                if distance <= ball.radius:
                    # Collision detected
                    normal_x, normal_y = self.line_normal(x1, y1, x2, y2)
                    dot_product = ball.vx * normal_x + ball.vy * normal_y
                    ball.vx -= 2 * dot_product * normal_x
                    ball.vy -= 2 * dot_product * normal_y

                    # Move the ball out of the wall
                    overlap = ball.radius - distance
                    ball.x += overlap * normal_x
                    ball.y += overlap * normal_y

                    # Calculate heptagon wall speed at collision point
                    wall_x_component, wall_y_component = self.calculate_wall_speed(
                        ball.x, ball.y, self.heptagon_center_x, self.heptagon_center_y
                    )
                    # Adjust ball speed based on heptagon wall speed
                    ball.vx += wall_x_component
                    ball.vy += wall_y_component

                    ball.angular_velocity += (
                        ball.vx * normal_y - ball.vy * normal_x
                    ) * 0.01  # Spin the ball upon collision

                    # Limit the bounce height (coefficient of restitution)
                    # the original bounce height is controlled by ball.vx and ball.vy, it is the kinetic energy transferred when bouncing
                    # the height of impact bounce should not exceed the radius of the heptagon, but higher than ball radius
                    velocity_magnitude = math.sqrt(ball.vx**2 + ball.vy**2)
                    max_velocity = math.sqrt(
                        2 * self.gravity * (self.heptagon_radius - self.ball_radius)
                    )  # max height equals to heptagon radius
                    # Scale the velocity by some factor to limit the bounce height
                    scale_factor = min(
                        1, max_velocity / velocity_magnitude
                    )  # bounce height should be higher than ball radius
                    ball.vx *= scale_factor
                    ball.vy *= scale_factor

    def handle_ball_collisions(self):
        for i in range(self.num_balls):
            for j in range(i + 1, self.num_balls):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                distance = math.sqrt(
                    (ball1.x - ball2.x) ** 2 + (ball1.y - ball2.y) ** 2
                )
                if distance <= ball1.radius + ball2.radius:
                    # Collision detected
                    normal_x = (ball2.x - ball1.x) / distance
                    normal_y = (ball2.y - ball1.y) / distance
                    tangent_x = -normal_y
                    tangent_y = normal_x

                    # Resolve velocities into normal and tangent components
                    v1n = ball1.vx * normal_x + ball1.vy * normal_y
                    v1t = ball1.vx * tangent_x + ball1.vy * tangent_y
                    v2n = ball2.vx * normal_x + ball2.vy * normal_y
                    v2t = ball2.vx * tangent_x + ball2.vy * tangent_y

                    # Apply conservation of momentum (assuming equal mass)
                    new_v1n = v2n
                    new_v2n = v1n

                    # Convert back to x and y components
                    ball1.vx = new_v1n * normal_x + v1t * tangent_x
                    ball1.vy = new_v1n * normal_y + v1t * tangent_y
                    ball2.vx = new_v2n * normal_x + v2t * tangent_x
                    ball2.vy = new_v2n * normal_y + v2t * tangent_y

                    # Separate the balls to avoid sticking
                    overlap = (ball1.radius + ball2.radius - distance) / 2
                    ball1.x -= overlap * normal_x
                    ball1.y -= overlap * normal_y
                    ball2.x += overlap * normal_x
                    ball2.y += overlap * normal_y

                    # Transfer angular momentum based on collision force
                    impulse = new_v1n - v1n
                    ball1.angular_velocity += impulse * 0.01
                    ball2.angular_velocity -= impulse * 0.01

    def rotate_heptagon(self):
        self.heptagon_angle += self.heptagon_rotation_speed / 100
        self.heptagon_angle %= 360
        points = []
        for i in range(7):
            angle = math.radians(self.heptagon_angle + i * 360 / 7)
            x = self.heptagon_center_x + self.heptagon_radius * math.cos(angle)
            y = self.heptagon_center_y + self.heptagon_radius * math.sin(angle)
            points.append(x)
            points.append(y)
        self.heptagon_points = [
            (points[i], points[i + 1]) for i in range(0, len(points), 2)
        ]

    def distance_to_line(self, point_x, point_y, line_x1, line_y1, line_x2, line_y2):
        # Calculate the distance from a point to a line segment
        dx = line_x2 - line_x1
        dy = line_y2 - line_y1
        if dx == 0 and dy == 0:
            return math.sqrt((point_x - line_x1) ** 2 + (point_y - line_y1) ** 2)

        t = ((point_x - line_x1) * dx + (point_y - line_y1) * dy) / (dx**2 + dy**2)
        t = max(0, min(1, t))  # Clamp t to be within the line segment

        closest_x = line_x1 + t * dx
        closest_y = line_y1 + t * dy

        return math.sqrt((point_x - closest_x) ** 2 + (point_y - closest_y) ** 2)

    def line_normal(self, x1, y1, x2, y2):
        # Calculate the normal vector of a line segment
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx**2 + dy**2)
        if length == 0:
            return 0, 0
        normal_x = -dy / length
        normal_y = dx / length
        return normal_x, normal_y

    def calculate_wall_speed(self, ball_x, ball_y, center_x, center_y):
        # Calculate the tangential velocity of the heptagon wall at the point of collision
        radius_vector_x = ball_x - center_x
        radius_vector_y = ball_y - center_y

        # Convert rotation speed from degrees per second to radians per second
        angular_speed_rad = math.radians(self.heptagon_rotation_speed / 100)

        # Calculate tangential velocity components
        wall_x_component = -angular_speed_rad * radius_vector_y
        wall_y_component = angular_speed_rad * radius_vector_x

        return wall_x_component, wall_y_component

    def draw(self):
        self.canvas.delete("all")
        self.draw_heptagon()

        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")

            # Draw the number on the ball, rotated
            angle_rad = math.radians(ball.rotation_angle)
            text_x = ball.x + ball.radius / 2 * math.cos(angle_rad)
            text_y = ball.y + ball.radius / 2 * math.sin(angle_rad)
            self.canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(ball.radius * 0.6)),
            )


root = tk.Tk()
bouncer = HeptagonBouncer(root)
root.mainloop()
