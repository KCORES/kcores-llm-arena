import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    number: int
    color: str
    rotation: float  # radians


@dataclass
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    rotation: float  # radians


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def collision_response(ball1: Ball, ball2: Ball):
    """Resolve collision between two balls."""
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)

    # Ensure the balls are actually colliding
    if dist < ball1.radius + ball2.radius:
        # Calculate the normal vector
        nx = dx / dist
        ny = dy / dist

        # Calculate the relative velocity
        dvx = ball1.vx - ball2.vx
        dvy = ball1.vy - ball2.vy

        # Calculate the dot product of the relative velocity and the normal vector
        dot_product = dvx * nx + dvy * ny

        # If the balls are moving away from each other, do nothing
        if dot_product > 0:
            return

        # Calculate the impulse
        impulse = (2 * dot_product) / (1 / 1 + 1 / 1)  # Assuming equal mass (mass = 1)

        # Apply the impulse
        ball1.vx -= impulse * nx
        ball1.vy -= impulse * ny
        ball2.vx += impulse * nx
        ball2.vy += impulse * ny


def bounce_off_heptagon(ball: Ball, heptagon: Heptagon):
    """Bounce a ball off the heptagon's sides."""
    for i in range(7):
        angle = 2 * math.pi * i / 7
        x1 = heptagon.center_x + heptagon.radius * math.cos(angle + heptagon.rotation)
        y1 = heptagon.center_y + heptagon.radius * math.sin(angle + heptagon.rotation)
        x2 = heptagon.center_x + heptagon.radius * math.cos(
            (angle + 1) * math.pi / 7 + heptagon.rotation
        )
        y2 = heptagon.center_y + heptagon.radius * math.sin(
            (angle + 1) * math.pi / 7 + heptagon.rotation
        )

        # Calculate the distance from the ball's center to the line segment
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            continue
        t = ((ball.x - x1) * dx + (ball.y - y1) * dy) / (dx * dx + dy * dy)

        # Clamp t to the range [0, 1]
        t = max(0, min(1, t))

        # Find the closest point on the line segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        # Calculate the distance from the ball's center to the closest point
        dist = distance(ball.x, ball.y, closest_x, closest_y)

        # If the distance is less than the ball's radius, the ball is colliding with the line segment
        if dist < ball.radius:
            # Calculate the normal vector
            nx = (closest_y - ball.y) / dist
            ny = (closest_x - ball.x) / dist

            # Calculate the relative velocity
            dot_product = ball.vx * nx + ball.vy * ny

            # If the ball is moving away from the line, do nothing
            if dot_product > 0:
                continue

            # Reflect the velocity
            ball.vx -= (
                2 * dot_product * nx * 0.8
            )  # 0.8 is the coefficient of restitution
            ball.vy -= 2 * dot_product * ny * 0.8


class BallSimulation:
    def __init__(self, master):
        self.master = master
        master.title("Bouncing Balls in a Spinning Heptagon")

        self.width = 800
        self.height = 600
        self.canvas = tk.Canvas(
            master, width=self.width, height=self.height, bg="black"
        )
        self.canvas.pack()

        self.balls: List[Ball] = []
        self.heptagon = Heptagon(self.width / 2, self.height / 2, 150, 0)
        self.colors = [
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

        self.gravity = 0.5
        self.friction = 0.98
        self.rotation_speed = 360 / (
            5 * 60
        )  # degrees per frame (5 seconds for 360 degrees)

        self.create_balls()

        self.animate()

    def create_balls(self):
        for i in range(20):
            ball = Ball(
                x=self.heptagon.center_x,
                y=self.heptagon.center_y,
                vx=np.random.uniform(-5, 5),
                vy=np.random.uniform(-5, 5),
                radius=10,
                number=i + 1,
                color=self.colors[i % len(self.colors)],
                rotation=np.random.uniform(0, 2 * math.pi),
            )
            self.balls.append(ball)

    def animate(self):
        self.heptagon.rotation += self.rotation_speed / 60
        self.heptagon.rotation %= 2 * math.pi

        for ball in self.balls:
            # Update velocity due to gravity and friction
            ball.vy += self.gravity
            ball.vx *= self.friction
            ball.vy *= self.friction

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Bounce off walls
            bounce_off_heptagon(ball, self.heptagon)

            # Keep balls within screen bounds (with minimal bounce)
            if ball.x - ball.radius < 0 or ball.x + ball.radius > self.width:
                ball.vx *= -0.7
            if ball.y - ball.radius < 0 or ball.y + ball.radius > self.height:
                ball.vy *= -0.7

            # Ball-ball collisions
            for other_ball in self.balls:
                if ball != other_ball:
                    collision_response(ball, other_ball)

            # Update rotation (simulating friction)
            ball.rotation += (ball.vx + ball.vy) * 0.02
            ball.rotation %= 2 * math.pi

        self.draw()
        self.master.after(16, self.animate)  # Roughly 60fps

    def draw(self):
        self.canvas.delete("all")

        # Draw Heptagon
        points = []
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.heptagon.rotation
            x = self.heptagon.center_x + self.heptagon.radius * math.cos(angle)
            y = self.heptagon.center_y + self.heptagon.radius * math.sin(angle)
            points.append(x)
            points.append(y)
        self.canvas.create_polygon(points, fill="", outline="white")

        # Draw Balls
        for ball in self.balls:
            x1 = ball.x - ball.radius
            y1 = ball.y - ball.radius
            x2 = ball.x + ball.radius
            y2 = ball.y + ball.radius
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color)

            # Add ball number
            self.canvas.create_text(ball.x, ball.y, text=str(ball.number), fill="black")


root = tk.Tk()
simulation = BallSimulation(root)
root.mainloop()
