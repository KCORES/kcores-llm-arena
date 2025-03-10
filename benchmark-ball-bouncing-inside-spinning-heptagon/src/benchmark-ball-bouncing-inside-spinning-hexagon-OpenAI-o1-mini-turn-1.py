import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple
import sys
import time

# Constants
WIDTH, HEIGHT = 800, 800
CENTER = (WIDTH // 2, HEIGHT // 2)
HEPTAGON_RADIUS = 300
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.99
SPIN_SPEED = 360 / 5  # degrees per second
NUM_BALLS = 20
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
    number: int
    color: str
    position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    spin: float = 0.0
    item: int = 0

    def update_position(self, dt: float):
        # Apply gravity
        self.velocity[1] += GRAVITY * dt
        # Apply friction
        self.velocity *= FRICTION
        # Update position
        self.position += self.velocity * dt
        # Update spin based on velocity
        self.spin += self.velocity[0] * dt


@dataclass
class Heptagon:
    radius: float
    center: Tuple[int, int]
    angle: float = 0.0  # in degrees
    vertices: List[Tuple[float, float]] = field(default_factory=list)

    def __post_init__(self):
        self.update_vertices()

    def update_vertices(self):
        self.vertices = []
        for i in range(7):
            theta = math.radians(self.angle + i * (360 / 7))
            x = self.center[0] + self.radius * math.cos(theta)
            y = self.center[1] + self.radius * math.sin(theta)
            self.vertices.append((x, y))

    def rotate(self, delta_angle: float):
        self.angle = (self.angle + delta_angle) % 360
        self.update_vertices()


class Simulation:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()
        self.heptagon = Heptagon(HEPTAGON_RADIUS, CENTER)
        self.balls: List[Ball] = []
        self.create_heptagon()
        self.create_balls()
        self.last_time = time.time()
        self.animate()

    def create_heptagon(self):
        points = []
        for vertex in self.heptagon.vertices:
            points.extend(vertex)
        self.heptagon_item = self.canvas.create_polygon(
            points, outline="black", fill="", width=2
        )

    def create_balls(self):
        for i in range(1, NUM_BALLS + 1):
            color = COLORS[i - 1]
            ball = Ball(number=i, color=color)
            ball.position = np.array(CENTER, dtype=float)
            angle = np.random.uniform(0, 2 * math.pi)
            speed = np.random.uniform(0, 5)
            ball.velocity = np.array([speed * math.cos(angle), speed * math.sin(angle)])
            ball.spin = 0.0
            # Draw ball
            x, y = ball.position
            ball.item = self.canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )
            # Draw number
            ball.text = self.canvas.create_text(
                x, y, text=str(ball.number), fill="white"
            )
            self.balls.append(ball)

    def animate(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.update_heptagon(dt)
        self.update_balls(dt)
        self.master.after(16, self.animate)  # Approximately 60 FPS

    def update_heptagon(self, dt):
        delta_angle = SPIN_SPEED * dt
        self.heptagon.rotate(delta_angle)
        # Update heptagon drawing
        self.canvas.delete(self.heptagon_item)
        points = []
        for vertex in self.heptagon.vertices:
            points.extend(vertex)
        self.heptagon_item = self.canvas.create_polygon(
            points, outline="black", fill="", width=2
        )

    def update_balls(self, dt):
        for ball in self.balls:
            ball.update_position(dt)
            self.handle_wall_collision(ball)
        self.handle_ball_collisions()
        self.redraw_balls()

    def handle_wall_collision(self, ball: Ball):
        # Calculate distance from center
        dist = np.linalg.norm(ball.position - np.array(self.heptagon.center))
        if dist + BALL_RADIUS > self.heptagon.radius:
            # Normalize the vector
            normal = (ball.position - np.array(self.heptagon.center)) / dist
            # Reflect velocity
            velocity_normal = np.dot(ball.velocity, normal)
            if velocity_normal > 0:
                ball.velocity -= 2 * velocity_normal * normal
            # Adjust position
            overlap = dist + BALL_RADIUS - self.heptagon.radius
            ball.position -= normal * overlap
            # Apply friction
            ball.velocity *= FRICTION

    def handle_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                delta_pos = ball2.position - ball1.position
                dist = np.linalg.norm(delta_pos)
                if dist < 2 * BALL_RADIUS:
                    # Normalize the delta position
                    if dist == 0:
                        dist = 0.1
                    normal = delta_pos / dist
                    # Relative velocity
                    relative_velocity = ball1.velocity - ball2.velocity
                    vel_along_normal = np.dot(relative_velocity, normal)
                    if vel_along_normal > 0:
                        continue
                    # Simple collision response
                    restitution = 0.9
                    impulse = -(1 + restitution) * vel_along_normal
                    impulse_vector = impulse * normal
                    ball1.velocity += impulse_vector
                    ball2.velocity -= impulse_vector
                    # Adjust positions to prevent overlap
                    overlap = 2 * BALL_RADIUS - dist
                    ball1.position -= normal * (overlap / 2)
                    ball2.position += normal * (overlap / 2)

    def redraw_balls(self):
        for ball in self.balls:
            x, y = ball.position
            self.canvas.coords(
                ball.item,
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
            )
            self.canvas.coords(ball.text, x, y)


def main():
    root = tk.Tk()
    root.title("旋转七边形内的弹跳球")
    sim = Simulation(root)
    root.mainloop()


if __name__ == "__main__":
    main()
