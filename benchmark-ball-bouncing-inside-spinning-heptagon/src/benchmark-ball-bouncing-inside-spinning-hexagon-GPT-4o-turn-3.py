import tkinter as tk
import numpy as np
import math
from dataclasses import dataclass
from typing import List

# Constants
NUM_BALLS = 20
BALL_RADIUS = 10
HEPTAGON_RADIUS = 200
SPIN_SPEED = 360 / 5  # degrees per second
GRAVITY = 0.1
FRICTION = 0.99
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
    number: int
    color: str
    spin: float = 0.0

    def update_position(self):
        self.vy += GRAVITY
        self.x += self.vx
        self.y += self.vy
        self.vx *= FRICTION
        self.vy *= FRICTION

    def draw(self, canvas: tk.Canvas):
        canvas.create_oval(
            self.x - BALL_RADIUS,
            self.y - BALL_RADIUS,
            self.x + BALL_RADIUS,
            self.y + BALL_RADIUS,
            fill=self.color,
            outline="",
        )
        canvas.create_text(self.x, self.y, text=str(self.number), fill="black")


def create_heptagon_points(center_x, center_y, radius, angle_offset):
    points = []
    for i in range(7):
        angle = math.radians(i * 360 / 7 + angle_offset)
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        points.append((x, y))
    return points


def check_collision(ball: Ball, heptagon_points):
    for i in range(len(heptagon_points)):
        p1 = heptagon_points[i]
        p2 = heptagon_points[(i + 1) % len(heptagon_points)]
        edge_vector = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        ball_vector = np.array([ball.x - p1[0], ball.y - p1[1]])
        edge_length = np.linalg.norm(edge_vector)
        edge_unit = edge_vector / edge_length
        projection_length = np.dot(ball_vector, edge_unit)
        closest_point = np.array(p1) + projection_length * edge_unit
        distance_to_edge = np.linalg.norm(np.array([ball.x, ball.y]) - closest_point)

        if distance_to_edge < BALL_RADIUS:
            normal = np.array([-edge_unit[1], edge_unit[0]])
            ball.vx, ball.vy = ball.vx - 2 * np.dot([ball.vx, ball.vy], normal) * normal
            ball.spin += 0.1 * np.linalg.norm([ball.vx, ball.vy])


def check_ball_collisions(balls: List[Ball]):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            dx = balls[j].x - balls[i].x
            dy = balls[j].y - balls[i].y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < 2 * BALL_RADIUS:
                nx, ny = dx / distance, dy / distance
                p = (
                    2
                    * (
                        balls[i].vx * nx
                        + balls[i].vy * ny
                        - balls[j].vx * nx
                        - balls[j].vy * ny
                    )
                    / 2
                )
                balls[i].vx -= p * nx
                balls[i].vy -= p * ny
                balls[j].vx += p * nx
                balls[j].vy += p * ny


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas = tk.Canvas(root, width=500, height=500, bg="white")
    canvas.pack()

    balls = [
        Ball(
            250,
            250,
            np.random.uniform(-2, 2),
            np.random.uniform(-2, 2),
            i + 1,
            COLORS[i],
        )
        for i in range(NUM_BALLS)
    ]
    angle_offset = 0

    def update():
        nonlocal angle_offset
        canvas.delete("all")
        angle_offset += SPIN_SPEED / 60
        heptagon_points = create_heptagon_points(
            250, 250, HEPTAGON_RADIUS, angle_offset
        )
        canvas.create_polygon(heptagon_points, outline="black", fill="", width=2)

        for ball in balls:
            ball.update_position()
            check_collision(ball, heptagon_points)
            ball.draw(canvas)

        check_ball_collisions(balls)
        root.after(16, update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
