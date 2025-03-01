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
        self.update()

    def create_balls(self):
        for i in range(NUM_BALLS):
            angle = 2 * math.pi * i / NUM_BALLS
            ball = Ball(
                x=200 + BALL_RADIUS * math.cos(angle),
                y=200 + BALL_RADIUS * math.sin(angle),
                vx=np.random.uniform(-2, 2),
                vy=np.random.uniform(-2, 2),
                number=i + 1,
                color=COLORS[i],
            )
            self.balls.append(ball)

    def update(self):
        self.canvas.delete("all")
        self.angle += SPIN_SPEED * (
            self.master.winfo_fpixels("1i") / 1000
        )  # Convert to milliseconds
        self.draw_heptagon()
        self.update_balls()
        self.master.after(10, self.update)

    def draw_heptagon(self):
        points = []
        for i in range(7):
            angle = self.angle + (2 * math.pi * i / 7)
            x = 200 + HEPTAGON_RADIUS * math.cos(angle)
            y = 200 + HEPTAGON_RADIUS * math.sin(angle)
            points.extend((x, y))
        self.canvas.create_polygon(points, outline="black", fill="", width=2)

    def update_balls(self):
        for ball in self.balls:
            ball.vy += GRAVITY  # Apply gravity
            ball.x += ball.vx
            ball.y += ball.vy

            # Bounce off the heptagon walls
            if not self.is_inside_heptagon(ball.x, ball.y):
                ball.vy = -ball.vy * FRICTION
                ball.y = max(ball.y, 0)  # Prevent going above the heptagon
                ball.x = min(max(ball.x, BALL_RADIUS), 400 - BALL_RADIUS)

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
            angle1 = self.angle + (2 * math.pi * i / 7)
            angle2 = self.angle + (2 * math.pi * (i + 1) % 7 / 7)
            x1 = 200 + HEPTAGON_RADIUS * math.cos(angle1)
            y1 = 200 + HEPTAGON_RADIUS * math.sin(angle1)
            x2 = 200 + HEPTAGON_RADIUS * math.cos(angle2)
            y2 = 200 + HEPTAGON_RADIUS * math.sin(angle2)
            if self.cross_product(x1, y1, x2, y2, x, y) < 0:
                return False
        return True

    def cross_product(self, x1, y1, x2, y2, x, y):
        return (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)


if __name__ == "__main__":
    root = tk.Tk()
    app = HeptagonBouncingBalls(root)
    root.mainloop()
