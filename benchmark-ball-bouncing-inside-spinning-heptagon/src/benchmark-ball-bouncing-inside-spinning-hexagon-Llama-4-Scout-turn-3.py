import tkinter as tk
import math
import random

WIDTH, HEIGHT = 800, 600
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 200
BALL_RADIUS = 10


class Ball:
    def __init__(self, canvas, x, y, vx, vy, color, number):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.color = color
        self.number = number
        self.id = self.canvas.create_oval(
            x - BALL_RADIUS,
            y - BALL_RADIUS,
            x + BALL_RADIUS,
            y + BALL_RADIUS,
            fill=color,
        )

    def update(self):
        self.x += self.vx
        self.y += self.vy
        self.vy += 0.1
        self.canvas.coords(
            self.id,
            self.x - BALL_RADIUS,
            self.y - BALL_RADIUS,
            self.x + BALL_RADIUS,
            self.y + BALL_RADIUS,
        )


class Heptagon:
    def __init__(self, canvas, x, y, radius):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.radius = radius
        self.angle = 0

    def draw(self):
        points = []
        for i in range(HEPTAGON_SIDES):
            angle = 2 * math.pi * i / HEPTAGON_SIDES + self.angle
            x = WIDTH // 2 + self.radius * 100 * math.cos(angle)
            y = HEIGHT // 2 + self.radius * 100 * math.sin(angle)
            points.append(x)
            points.append(y)
        self.canvas.create_polygon(points, fill="", outline="black")


class Simulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.balls = []
        self.heptagon_angle = 0
        self.heptagon = Heptagon(self.canvas, WIDTH // 2, HEIGHT // 2, 200)

        for i in range(20):
            ball = Ball(
                self.canvas,
                WIDTH // 2 + random.uniform(-100, 100),
                HEIGHT // 2 + random.uniform(-100, 100),
                random.uniform(-2, 2),
                random.uniform(-2, 0),
                random.choice(
                    [
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
                ),
                i + 1,
            )
            self.balls.append(ball)

    def update(self):
        self.canvas.delete("all")
        for ball in self.balls:
            ball.x += ball.vx
            ball.y += ball.vy
            ball.vy += 0.1
            if (
                math.hypot(ball.x - WIDTH // 2, ball.y - HEIGHT // 2)
                > 200 - BALL_RADIUS
            ):
                self.canvas.create_oval(
                    ball.x - BALL_RADIUS,
                    ball.y - BALL_RADIUS,
                    ball.x + BALL_RADIUS,
                    ball.y + BALL_RADIUS,
                    fill=ball.color,
                )
            else:
                self.canvas.create_oval(
                    ball.x - BALL_RADIUS,
                    ball.y - BALL_RADIUS,
                    ball.x + BALL_RADIUS,
                    ball.y + BALL_RADIUS,
                    fill=ball.color,
                )

        self.root.after(16, self.update)


root = tk.Tk()
simulation = Simulation(root)
simulation.update()
root.mainloop()
