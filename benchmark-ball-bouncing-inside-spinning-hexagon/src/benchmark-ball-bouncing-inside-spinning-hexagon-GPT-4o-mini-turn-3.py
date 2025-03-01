import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
NUM_BALLS = 20
BALL_RADIUS = 10
HEPTAGON_RADIUS = 150
GRAVITY = 0.5
FRICTION = 0.99
SPIN_SPEED = 360 / 5000  # degrees per millisecond

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
    number: int
    color: str


class HeptagonBouncingBalls:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=400, height=400, bg="white")
        self.canvas.pack()
        self.balls: List[Ball] = []
        self.angle = 0
        self.create_balls()
        self.animate()

    def create_balls(self):
        for i in range(NUM_BALLS):
            ball = Ball(
                x=200,  # Start from the center
                y=200,
                vx=np.random.uniform(-2, 2),
                vy=np.random.uniform(-2, 2),
                number=i + 1,
                color=COLORS[i],
            )
            self.balls.append(ball)

    def draw_heptagon(self):
        points = []
        for i in range(7):
            angle = math.radians(self.angle + (i * 360 / 7))
            x = 200 + HEPTAGON_RADIUS * math.cos(angle)
            y = 200 + HEPTAGON_RADIUS * math.sin(angle)
            points.append((x, y))
        return points

    def animate(self):
        self.canvas.delete("all")
        self.angle += SPIN_SPEED
        heptagon_points = self.draw_heptagon()
        self.canvas.create_polygon(heptagon_points, outline="black", fill="", width=2)

        for ball in self.balls:
            ball.vy += GRAVITY  # Apply gravity
            ball.x += ball.vx
            ball.y += ball.vy

            # Check for wall collisions
            if not self.is_inside_heptagon(ball.x, ball.y):
                ball.vx *= -FRICTION
                ball.vy *= -FRICTION
                ball.x = max(min(ball.x, 390), 10)  # Keep within bounds
                ball.y = max(min(ball.y, 390), 10)

            # Draw the ball
            self.canvas.create_oval(
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS,
                ball.y + BALL_RADIUS,
                fill=ball.color,
            )
            self.canvas.create_text(ball.x, ball.y, text=str(ball.number), fill="black")

        self.master.after(20, self.animate)

    def is_inside_heptagon(self, x, y):
        # Check if the point (x, y) is inside the heptagon
        angle = math.radians(self.angle)
        for i in range(7):
            next_i = (i + 1) % 7
            x1 = 200 + HEPTAGON_RADIUS * math.cos(angle + (i * 360 / 7))
            y1 = 200 + HEPTAGON_RADIUS * math.sin(angle + (i * 360 / 7))
            x2 = 200 + HEPTAGON_RADIUS * math.cos(angle + (next_i * 360 / 7))
            y2 = 200 + HEPTAGON_RADIUS * math.sin(angle + (next_i * 360 / 7))
            if self.line_intersects_circle(x1, y1, x2, y2, x, y, BALL_RADIUS):
                return False
        return True

    def line_intersects_circle(self, x1, y1, x2, y2, cx, cy, radius):
        # Check if the line segment (x1, y1) to (x2, y2) intersects with the circle (cx, cy, radius)
        # Using the distance from point to line segment formula
        A = x2 - x1
        B = y2 - y1
        C = cx - x1
        D = cy - y1

        dot = A * C + B * D
        len_sq = A * A + B * B
        param = -1
        if len_sq != 0:
            param = dot / len_sq

        if param < 0:
            closest_x = x1
            closest_y = y1
        elif param > 1:
            closest_x = x2
            closest_y = y2
        else:
            closest_x = x1 + param * A
            closest_y = y1 + param * B

        distance = math.sqrt((closest_x - cx) ** 2 + (closest_y - cy) ** 2)
        return distance < radius


if __name__ == "__main__":
    root = tk.Tk()
    app = HeptagonBouncingBalls(root)
    root.mainloop()
