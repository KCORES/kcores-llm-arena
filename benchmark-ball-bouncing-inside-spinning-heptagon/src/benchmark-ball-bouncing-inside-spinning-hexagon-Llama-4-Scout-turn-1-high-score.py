import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 600
HEPTAGON_SIZE = 400
BALL_RADIUS = 10
GRAVITY = 0.1
FRICTION = 0.98
BOUNCE_FRICTION = -0.8
SPIN_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds


class Ball:
    def __init__(self, canvas, x, y, color, number):
        self.canvas = canvas
        self.id = self.canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=color,
        )
        self.text_id = self.canvas.create_text(
            x, y, text=str(number), font=("Arial", 12)
        )
        self.x = x
        self.y = y
        self.vx = np.random.uniform(-2, 2)
        self.vy = 0
        self.color = color
        self.number = number

    def update(self):
        self.x += self.vx
        self.y += self.vy
        self.vy += GRAVITY
        self.canvas.coords(
            self.id,
            self.x - BALL_RADIUS,
            self.y - BALL_RADIUS,
            self.x + BALL_RADIUS,
            self.y + BALL_RADIUS,
        )
        self.canvas.coords(self.text_id, self.x, self.y)

    def bounce(self, normal_x, normal_y):
        v_normal = self.vx * normal_x + self.vy * normal_y
        v_tangent = -self.vx * normal_y + self.vy * normal_x
        v_normal = v_normal * BOUNCE_FRICTION
        self.vx = v_normal * normal_x + v_tangent * -normal_y
        self.vy = v_normal * normal_y + v_tangent * normal_x


class Heptagon:
    def __init__(self, canvas):
        self.canvas = canvas
        self.angle = 0
        self.points = []
        for i in range(7):
            angle = 2 * math.pi * i / 7
            x = WIDTH / 2 + HEPTAGON_SIZE * math.cos(angle)
            y = HEIGHT / 2 + HEPTAGON_SIZE * math.sin(angle)
            self.points.append((x, y))

    def update(self):
        self.angle += SPIN_SPEED / 60
        self.points = []
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.angle
            x = WIDTH / 2 + HEPTAGON_SIZE * math.cos(angle)
            y = HEIGHT / 2 + HEPTAGON_SIZE * math.sin(angle)
            self.points.append((x, y))
        self.canvas.delete("heptagon")
        self.canvas.create_polygon(
            self.points, fill="", outline="black", width=2, tag="heptagon"
        )


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
    canvas.pack()

    heptagon = Heptagon(canvas)

    balls = []
    colors = [
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
    for i in range(20):
        ball = Ball(canvas, WIDTH / 2, HEIGHT / 2, colors[i], i + 1)
        balls.append(ball)

    def update():
        heptagon.update()
        for ball in balls:
            ball.update()
            # boundary collision
            dx = ball.x - WIDTH / 2
            dy = ball.y - HEIGHT / 2
            distance = math.sqrt(dx**2 + dy**2)
            if distance > HEPTAGON_SIZE - BALL_RADIUS:
                normal_x = dx / distance
                normal_y = dy / distance
                tangent_x = -normal_y
                tangent_y = normal_x
                v_normal = ball.vx * normal_x + ball.vy * normal_y
                v_tangent = ball.vx * tangent_x + ball.vy * tangent_y
                v_normal = v_normal * BOUNCE_FRICTION
                ball.vx = v_normal * normal_x + v_tangent * tangent_x
                ball.vy = v_normal * normal_y + v_tangent * tangent_y
            # ball collision
            for other in balls:
                if other != ball:
                    dx = ball.x - other.x
                    dy = ball.y - other.y
                    distance = math.sqrt(dx**2 + dy**2)
                    if distance < 2 * BALL_RADIUS:
                        ball.vx, other.vx = other.vx, ball.vx
                        ball.vy, other.vy = other.vy, ball.vy
        root.after(16, update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
