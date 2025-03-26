import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
NUMBER_OF_BALLS = 20
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300
GRAVITY = 0.1
FRICTION = 0.99
SPIN_TIME = 5  # 360 degrees in 5 seconds
SPIN_SPEED = 360 / (SPIN_TIME * 1000 / 16)  # 16 ms per frame

COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51",
    "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800",
    "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61",
    "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    number: int
    color: str
    spin: float = 0

    def draw(self, canvas):
        canvas.create_oval(self.x - BALL_RADIUS, self.y - BALL_RADIUS,
                           self.x + BALL_RADIUS, self.y + BALL_RADIUS,
                           fill=self.color)
        canvas.create_text(self.x, self.y, text=str(self.number))


def get_heptagon_points(angle=0):
    points = []
    for i in range(7):
        theta = math.radians(360 / 7 * i + angle)
        x = CENTER_X + HEPTAGON_RADIUS * math.cos(theta)
        y = CENTER_Y + HEPTAGON_RADIUS * math.sin(theta)
        points.extend([x, y])
    return points


def draw_heptagon(canvas, angle=0):
    points = get_heptagon_points(angle)
    canvas.create_polygon(points, outline="black", width=2)


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def line_point_distance(x1, y1, x2, y2, px, py):
    numerator = abs((y2 - y1) * px - (x2 - x1) * py + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
    return numerator / denominator


def ball_wall_collision(ball, heptagon_angle):
    points = get_heptagon_points(heptagon_angle)
    for i in range(7):
        x1, y1 = points[2 * i], points[2 * i + 1]
        x2, y2 = points[(2 * (i + 1)) % 14], points[(2 * (i + 1) + 1) % 14]
        dist = line_point_distance(x1, y1, x2, y2, ball.x, ball.y)
        if dist <= BALL_RADIUS:
            # Normal vector of the line
            dx = x2 - x1
            dy = y2 - y1
            normal_x = -dy
            normal_y = dx
            length = math.sqrt(normal_x ** 2 + normal_y ** 2)
            normal_x /= length
            normal_y /= length

            # Relative velocity
            dot_product = ball.vx * normal_x + ball.vy * normal_y
            ball.vx -= 2 * dot_product * normal_x
            ball.vy -= 2 * dot_product * normal_y

            # Apply friction
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            break


def ball_ball_collision(balls):
    for i in range(NUMBER_OF_BALLS):
        for j in range(i + 1, NUMBER_OF_BALLS):
            dist = distance(balls[i].x, balls[i].y, balls[j].x, balls[j].y)
            if dist <= 2 * BALL_RADIUS:
                dx = balls[j].x - balls[i].x
                dy = balls[j].y - balls[i].y
                normal_x = dx / dist
                normal_y = dy / dist

                v1n = balls[i].vx * normal_x + balls[i].vy * normal_y
                v2n = balls[j].vx * normal_x + balls[j].vy * normal_y

                v1t = balls[i].vx - v1n * normal_x
                v2t = balls[j].vx - v2n * normal_x

                v1n_new = v2n
                v2n_new = v1n

                balls[i].vx = v1t + v1n_new * normal_x
                balls[i].vy = v1t + v1n_new * normal_y
                balls[j].vx = v2t + v2n_new * normal_x
                balls[j].vy = v2t + v2n_new * normal_y

                # Apply friction
                balls[i].vx *= FRICTION
                balls[i].vy *= FRICTION
                balls[j].vx *= FRICTION
                balls[j].vy *= FRICTION


def update(canvas, balls, heptagon_angle):
    canvas.delete("all")
    draw_heptagon(canvas, heptagon_angle)

    for ball in balls:
        ball.vy += GRAVITY
        ball.x += ball.vx
        ball.y += ball.vy
        ball_wall_collision(ball, heptagon_angle)
        ball.draw(canvas)

    ball_ball_collision(balls)

    heptagon_angle = (heptagon_angle + SPIN_SPEED) % 360
    canvas.after(16, update, canvas, balls, heptagon_angle)


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    balls = [
        Ball(x=CENTER_X, y=CENTER_Y, vx=0, vy=0, number=i + 1, color=COLORS[i])
        for i in range(NUMBER_OF_BALLS)
    ]

    heptagon_angle = 0
    update(canvas, balls, heptagon_angle)

    root.mainloop()


if __name__ == "__main__":
    main()
