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
                x=200,  # Center of the heptagon
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
            angle = self.angle + i * (360 / 7)
            x = 200 + HEPTAGON_RADIUS * math.cos(math.radians(angle))
            y = 200 + HEPTAGON_RADIUS * math.sin(math.radians(angle))
            points.append((x, y))
        self.canvas.create_polygon(points, outline="black", fill="", width=2)

    def animate(self):
        self.canvas.delete("all")
        self.draw_heptagon()
        self.update_balls()
        self.angle += SPIN_SPEED
        self.master.after(10, self.animate)

    def update_balls(self):
        for ball in self.balls:
            ball.vy += GRAVITY  # Apply gravity
            ball.x += ball.vx
            ball.y += ball.vy

            # Check for collision with heptagon walls
            if not self.is_inside_heptagon(ball.x, ball.y):
                ball.vx *= -FRICTION
                ball.vy *= -FRICTION
                ball.x = max(min(ball.x, 390), 10)  # Keep within canvas
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

    def is_inside_heptagon(self, x, y):
        # Check if the point (x, y) is inside the heptagon
        for i in range(7):
            angle = self.angle + i * (360 / 7)
            next_angle = self.angle + (i + 1) * (360 / 7)
            x1 = 200 + HEPTAGON_RADIUS * math.cos(math.radians(angle))
            y1 = 200 + HEPTAGON_RADIUS * math.sin(math.radians(angle))
            x2 = 200 + HEPTAGON_RADIUS * math.cos(math.radians(next_angle))
            y2 = 200 + HEPTAGON_RADIUS * math.sin(math.radians(next_angle))
            if self.cross_product(x1, y1, x2, y2, x, y) < 0:
                return False
        return True

    def cross_product(self, x1, y1, x2, y2, x, y):
        return (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)


if __name__ == "__main__":
    root = tk.Tk()
    app = HeptagonBouncingBalls(root)
    root.mainloop()
