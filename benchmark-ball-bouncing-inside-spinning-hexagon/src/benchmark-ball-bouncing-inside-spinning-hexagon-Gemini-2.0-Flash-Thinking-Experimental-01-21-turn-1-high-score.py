import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Ball:
    id: int
    x: float
    y: float
    radius: float
    color: str
    vx: float = 0.0
    vy: float = 0.0
    spin_angle: float = 0.0
    spin_speed: float = 0.0


@dataclass
class Point:
    x: float
    y: float


def distance(p1: Point, p2: Point) -> float:
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def normalize_vector(v: np.array) -> np.array:
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def rotate_point(point: Point, center: Point, angle_rad: float) -> Point:
    """Rotate a point around a center point by a given angle in radians."""
    dx = point.x - center.x
    dy = point.y - center.y
    rotated_x = center.x + dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
    rotated_y = center.y + dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
    return Point(rotated_x, rotated_y)


class Heptagon:
    def __init__(self, center_x: float, center_y: float, radius: float):
        self.center = Point(center_x, center_y)
        self.radius = radius
        self.rotation_angle = 0.0  # in radians
        self.rotation_speed = math.radians(
            360 / 5
        )  # 360 degrees per 5 seconds in radians
        self.vertices = self._calculate_vertices()

    def _calculate_vertices(self) -> List[Point]:
        vertices = []
        for i in range(7):
            angle = 2 * math.pi / 7 * i + self.rotation_angle
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)
            vertices.append(Point(x, y))
        return vertices

    def update_rotation(self, dt: float):
        self.rotation_angle += self.rotation_speed * dt
        self.vertices = self._calculate_vertices()

    def draw(self, canvas: tk.Canvas):
        points = []
        for vertex in self.vertices:
            points.append(vertex.x)
            points.append(vertex.y)
        canvas.create_polygon(points, outline="black", fill="")


class Simulation:
    def __init__(self, master: tk.Tk):
        self.master = master
        master.title("Bouncing Balls in Spinning Heptagon")

        self.canvas_width = 800
        self.canvas_height = 600
        self.canvas = tk.Canvas(
            master, width=self.canvas_width, height=self.canvas_height, bg="white"
        )
        self.canvas.pack()

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
        self.num_balls = 20
        self.ball_radius = 20
        self.balls: List[Ball] = []
        self.heptagon_radius = 250
        self.heptagon = Heptagon(
            self.canvas_width / 2, self.canvas_height / 2, self.heptagon_radius
        )

        for i in range(self.num_balls):
            self.balls.append(
                Ball(
                    i + 1,
                    self.canvas_width / 2,
                    self.canvas_height / 2,
                    self.ball_radius,
                    self.colors[i],
                )
            )

        self.gravity = 981  # pixels per second squared
        self.friction = 0.01  # velocity reduction per update for linear motion
        self.spin_friction = 0.005  # spin speed reduction per update
        self.time_step = 0.016  # seconds per frame (approximately 60 FPS)
        self.ball_bounce_factor = 0.7
        self.wall_bounce_factor = 0.6
        self.ball_spin_increase_on_collision = 5.0  # degrees per second

        self.animate()

    def animate(self):
        self.canvas.delete("all")
        self.heptagon.update_rotation(self.time_step)
        self.heptagon.draw(self.canvas)

        for i in range(self.num_balls):
            ball = self.balls[i]

            # Apply gravity
            ball.vy += self.gravity * self.time_step

            # Apply friction to linear motion
            ball.vx *= 1 - self.friction
            ball.vy *= 1 - self.friction

            # Apply friction to spin
            ball.spin_speed *= 1 - self.spin_friction

            # Update position
            ball.x += ball.vx * self.time_step
            ball.y += ball.vy * self.time_step
            ball.spin_angle += math.radians(ball.spin_speed) * self.time_step

            # Wall collision detection and response
            for j in range(7):
                p1 = self.heptagon.vertices[j]
                p2 = self.heptagon.vertices[(j + 1) % 7]

                v_edge = np.array([p2.x - p1.x, p2.y - p1.y])
                v_ball_to_p1 = np.array([p1.x - ball.x, p1.y - ball.y])
                v_ball_to_p2 = np.array([p2.x - ball.x, p2.y - ball.y])

                edge_len_sq = np.dot(v_edge, v_edge)
                param = np.dot(v_ball_to_p1, -v_edge) / edge_len_sq
                closest_point_on_edge_x = p1.x + param * v_edge[0]
                closest_point_on_edge_y = p1.y + param * v_edge[1]

                closest_point = Point(closest_point_on_edge_x, closest_point_on_edge_y)

                if 0 <= param <= 1:  # Closest point is on the segment
                    dist_sq = (ball.x - closest_point.x) ** 2 + (
                        ball.y - closest_point.y
                    ) ** 2
                    if dist_sq < ball.radius**2:
                        normal_vector = normalize_vector(
                            np.array(
                                [ball.x - closest_point.x, ball.y - closest_point.y]
                            )
                        )
                        ball_velocity_vector = np.array([ball.vx, ball.vy])

                        # Reflection
                        v_normal_component = np.dot(ball_velocity_vector, normal_vector)
                        ball_velocity_vector -= 2 * v_normal_component * normal_vector

                        ball.vx = ball_velocity_vector[0] * self.wall_bounce_factor
                        ball.vy = ball_velocity_vector[1] * self.wall_bounce_factor

                        # Add spin based on collision direction
                        tangent_vector = normalize_vector(
                            np.array([-normal_vector[1], normal_vector[0]])
                        )  # Tangent vector
                        if np.dot(ball_velocity_vector, tangent_vector) > 0:
                            ball.spin_speed += self.ball_spin_increase_on_collision
                        else:
                            ball.spin_speed -= self.ball_spin_increase_on_collision

            # Ball-ball collision detection and response
            for j in range(i + 1, self.num_balls):
                ball2 = self.balls[j]
                dist_balls = distance(Point(ball.x, ball.y), Point(ball2.x, ball2.y))
                if dist_balls < 2 * ball.radius:
                    # Collision detected
                    normal_vector = normalize_vector(
                        np.array([ball2.x - ball.x, ball2.y - ball.y])
                    )
                    tangent_vector = np.array([-normal_vector[1], normal_vector[0]])

                    v1 = np.array([ball.vx, ball.vy])
                    v2 = np.array([ball2.vx, ball2.vy])

                    v1n = np.dot(v1, normal_vector)
                    v1t = np.dot(v1, tangent_vector)
                    v2n = np.dot(v2, normal_vector)
                    v2t = np.dot(v2, tangent_vector)

                    v1n_new = v2n
                    v2n_new = v1n

                    v1n_vec_new = v1n_new * normal_vector
                    v1t_vec = v1t * tangent_vector
                    v2n_vec_new = v2n_new * normal_vector
                    v2t_vec = v2t * tangent_vector

                    v1_final = v1n_vec_new + v1t_vec
                    v2_final = v2n_vec_new + v2t_vec

                    ball.vx = v1_final[0] * self.ball_bounce_factor
                    ball.vy = v1_final[1] * self.ball_bounce_factor
                    ball2.vx = v2_final[0] * self.ball_bounce_factor
                    ball2.vy = v2_final[1] * self.ball_bounce_factor

                    # Add spin based on collision direction
                    if np.dot(v1 - v2, tangent_vector) > 0:
                        ball.spin_speed += self.ball_spin_increase_on_collision
                        ball2.spin_speed -= self.ball_spin_increase_on_collision
                    else:
                        ball.spin_speed -= self.ball_spin_increase_on_collision
                        ball2.spin_speed += self.ball_spin_increase_on_collision

            # Draw ball
            self.draw_ball(ball, self.canvas)

        self.master.after(int(self.time_step * 1000), self.animate)

    def draw_ball(self, ball: Ball, canvas: tk.Canvas):
        x0 = ball.x - ball.radius
        y0 = ball.y - ball.radius
        x1 = ball.x + ball.radius
        y1 = ball.y + ball.radius
        canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="black")

        # Draw number with rotation
        text_x = ball.x
        text_y = ball.y
        angle_deg = math.degrees(ball.spin_angle)
        canvas.create_text(
            text_x,
            text_y,
            text=str(ball.id),
            fill="white",
            font=("Arial", int(ball.radius * 0.6), "bold"),
            angle=angle_deg,
        )


if __name__ == "__main__":
    root = tk.Tk()
    sim = Simulation(root)
    root.mainloop()
