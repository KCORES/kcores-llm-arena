import tkinter as tk
import math
import random

# Window size
WIDTH, HEIGHT = 800, 600

# Heptagon properties
HEPTAGON_SIDES = 7
HEPTAGON_RADIUS = 200

# Ball properties
BALL_RADIUS = 10
GRAVITY = 0.1
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


class Ball:
    def __init__(self, canvas, x, y, vx, vy, radius, color, number):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.color = color
        self.number = number


class Heptagon:
    def __init__(self, canvas, x, y):
        self.canvas = canvas
        self.x = x
        self.y = y
        self.angle = 0
        self.angular_velocity = 2 * math.pi / 50  # 360 degrees / 5 seconds

    def draw(self):
        points = []
        for i in range(HEPTAGON_SIDES):
            angle = self.angle + 2 * math.pi * i / HEPTAGON_SIDES
            px = WIDTH / 2 + 200 * math.cos(angle)
            py = HEIGHT / 2 + 200 * math.sin(angle)
            points.append(px)
            points.append(py)
        self.canvas.create_polygon(points, fill="", outline="black", width=2)


class Simulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(self.root, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.balls = []
        self.heptagon = Heptagon(self.canvas, WIDTH / 2, HEIGHT / 2)
        for i in range(20):
            ball = Ball(
                self.canvas,
                WIDTH / 2,
                HEIGHT / 2,
                random.uniform(-2, 2),
                random.uniform(-2, 0),
                BALL_RADIUS,
                random.choice(colors),
                i + 1,
            )
            self.balls.append(ball)

    def run(self):
        self.update()

    def update(self):
        self.canvas.delete("all")
        self.heptagon.angle += 0.01
        for ball in self.balls:
            ball.x += ball.vx
            ball.y += ball.vy
            ball.vy += GRAVITY / 10

            # collision with heptagon
            dx = ball.x - WIDTH / 2
            dy = ball.y - HEIGHT / 2
            distance = math.sqrt(dx**2 + dy**2)
            if distance > HEPTAGON_RADIUS - BALL_RADIUS:
                normal_x = dx / distance
                normal_y = dy / distance
                tangent_x = -normal_y
                tangent_y = normal_x
                v_normal = ball.vx * normal_x + ball.vy * normal_y
                v_tangent = ball.vx * tangent_x + ball.vy * tangent_y
                v_normal *= 0.8  # friction
                ball.vx = v_normal * normal_x + v_tangent * tangent_x
                ball.vy = v_normal * normal_y + v_tangent * tangent_y

            # draw ball
            self.canvas.create_oval(
                ball.x - BALL_RADIUS,
                ball.y - BALL_RADIUS,
                ball.x + BALL_RADIUS,
                ball.y + BALL_RADIUS,
                fill=ball.color,
            )
            self.canvas.create_text(ball.x, ball.y, text=str(ball.number), fill="black")

        self.root.after(16, self.update)


root = tk.Tk()
simulation = Simulation(root)


def main():
    for i, ball in enumerate(simulation.balls):
        ball.id = simulation.canvas.create_oval(
            ball.x - BALL_RADIUS,
            ball.y - BALL_RADIUS,
            ball.x + BALL_RADIUS,
            ball.y + BALL_RADIUS,
            fill=ball.color,
        )
    simulation.update()


root.mainloop()
