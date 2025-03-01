import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 600
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 250
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.99
ROTATION_SPEED = 360 / 5  # degrees per second
UPDATE_DELAY = 16  # approx 60 FPS
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
    number: int
    color: str
    position: np.ndarray
    velocity: np.ndarray
    spin: float
    id_circle: int = None
    id_text: int = None

    def update_position(self):
        self.velocity[1] += GRAVITY
        self.velocity *= FRICTION
        self.position += self.velocity

    def update_spin(self):
        self.spin *= FRICTION


@dataclass
class Heptagon:
    center: np.ndarray
    radius: float
    angle: float = 0  # in degrees

    def rotate(self, delta_time):
        self.angle = (self.angle + ROTATION_SPEED * delta_time) % 360

    def get_vertices(self):
        vertices = []
        for i in range(HEPTAGON_SIDES):
            theta = math.radians(self.angle + i * 360 / HEPTAGON_SIDES)
            x = self.center[0] + self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            vertices.append((x, y))
        return vertices


class BouncingBallsApp:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(
            master, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white"
        )
        self.canvas.pack()
        self.heptagon = Heptagon(
            np.array([WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2]), HEPTAGON_RADIUS
        )
        self.balls: List[Ball] = []
        self.create_heptagon()
        self.create_balls()
        self.last_time = None
        self.animate()

    def create_heptagon(self):
        vertices = self.heptagon.get_vertices()
        self.heptagon_id = self.canvas.create_polygon(
            vertices, outline="black", fill="", width=2
        )

    def create_balls(self):
        for i in range(20):
            position = self.heptagon.center.copy()
            velocity = np.array([0.0, 0.0])
            spin = 0.0
            ball = Ball(
                number=i + 1,
                color=BALL_COLORS[i],
                position=position,
                velocity=velocity,
                spin=spin,
            )
            x, y = position
            ball.id_circle = self.canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )
            ball.id_text = self.canvas.create_text(
                x, y, text=str(ball.number), fill="black"
            )
            self.balls.append(ball)

    def animate(self):
        current_time = self.master.after_info()
        if self.last_time is None:
            delta_time = 0
        else:
            delta_time = UPDATE_DELAY / 1000  # seconds
        self.last_time = current_time

        self.update_physics(delta_time)
        self.update_graphics()
        self.master.after(UPDATE_DELAY, self.animate)

    def update_physics(self, delta_time):
        self.heptagon.rotate(delta_time)
        self.handle_collisions()
        for ball in self.balls:
            ball.update_position()
            ball.update_spin()

    def update_graphics(self):
        # Update heptagon
        vertices = self.heptagon.get_vertices()
        self.canvas.coords(
            self.heptagon_id, *[coord for vertex in vertices for coord in vertex]
        )

        # Update balls
        for ball in self.balls:
            x, y = ball.position
            self.canvas.coords(
                ball.id_circle,
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
            )
            self.canvas.coords(ball.id_text, x, y)

    def handle_collisions(self):
        # Collision with walls
        heptagon_vertices = self.heptagon.get_vertices()
        for ball in self.balls:
            # Find the closest edge
            closest_dist = float("inf")
            closest_edge = None
            for i in range(HEPTAGON_SIDES):
                p1 = np.array(heptagon_vertices[i])
                p2 = np.array(heptagon_vertices[(i + 1) % HEPTAGON_SIDES])
                dist, _ = self.distance_point_to_segment(ball.position, p1, p2)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_edge = (p1, p2)
            if closest_dist <= BALL_RADIUS:
                # Reflect velocity
                p1, p2 = closest_edge
                edge_vec = p2 - p1
                edge_normal = np.array([-edge_vec[1], edge_vec[0]])
                edge_normal /= np.linalg.norm(edge_normal)
                vel_dot_norm = np.dot(ball.velocity, edge_normal)
                if vel_dot_norm < 0:
                    ball.velocity -= 2 * vel_dot_norm * edge_normal

        # Collision between balls
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                delta_pos = ball2.position - ball1.position
                distance = np.linalg.norm(delta_pos)
                if distance < 2 * BALL_RADIUS:
                    # Simple elastic collision
                    if distance == 0:
                        distance = 0.1
                    normal = delta_pos / distance
                    relative_velocity = ball1.velocity - ball2.velocity
                    vel_along_normal = np.dot(relative_velocity, normal)
                    if vel_along_normal > 0:
                        continue
                    ball1.velocity -= vel_along_normal * normal
                    ball2.velocity += vel_along_normal * normal
                    # Adjust positions
                    overlap = 2 * BALL_RADIUS - distance
                    ball1.position -= normal * (overlap / 2)
                    ball2.position += normal * (overlap / 2)

    def distance_point_to_segment(self, point, seg_a, seg_b):
        # Calculate distance from point to segment AB
        seg_vec = seg_b - seg_a
        point_vec = point - seg_a
        seg_len = np.dot(seg_vec, seg_vec)
        if seg_len == 0:
            return np.linalg.norm(point_vec), seg_a
        t = np.dot(point_vec, seg_vec) / seg_len
        t = max(0, min(1, t))
        projection = seg_a + t * seg_vec
        distance = np.linalg.norm(point - projection)
        return distance, projection


def main():
    root = tk.Tk()
    root.title("20 Bouncing Balls in Spinning Heptagon")
    app = BouncingBallsApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
