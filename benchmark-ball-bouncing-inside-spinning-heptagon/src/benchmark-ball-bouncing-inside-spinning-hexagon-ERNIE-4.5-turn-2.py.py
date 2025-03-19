import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class Ball:
    radius: float
    number: int
    color: str
    x: float
    y: float
    vx: float
    vy: float
    rotation: float
    rotation_speed: float

def heptagon_wall_collision(ball, heptagon_radius, center_x, center_y, heptagon_rotation):
    # Rotate ball position back to heptagon's frame of reference
    relative_x = ball.x - center_x
    relative_y = ball.y - center_y
    angle = -math.radians(heptagon_rotation)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    rotated_x = relative_x * cos_a - relative_y * sin_a
    rotated_y = relative_x * sin_a + relative_y * cos_a

    # Check collision with heptagon walls
    apothem = heptagon_radius * math.cos(math.pi / 7)  # Approximation for inscribed circle radius
    distance_from_center = math.sqrt(rotated_x**2 + rotated_y**2)
    if distance_from_center + ball.radius > heptagon_radius:
        # Normalize the vector to the point of impact
        normal_x = rotated_x / distance_from_center
        normal_y = rotated_y / distance_from_center
        # Reflect velocity
        dot = ball.vx * normal_x + ball.vy * normal_y
        ball.vx -= 2 * dot * normal_x
        ball.vy -= 2 * dot * normal_y
        # Apply some friction (damping)
        ball.vx *= 0.95
        ball.vy *= 0.95
        # Move the ball out of the wall slightly to avoid sticking
        overlap = (distance_from_center + ball.radius) - heptagon_radius
        ball.x -= overlap * normal_x * cos_a - overlap * normal_y * sin_a
        ball.y -= overlap * normal_x * sin_a + overlap * normal_y * cos_a

def ball_collision(ball1, ball2):
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    distance = math.sqrt(dx**2 + dy**2)
    if distance < 2 * ball1.radius:
        # Normalize the vector
        nx = dx / distance
        ny = dy / distance
        # Project velocities onto the normal
        dvx = ball2.vx - ball1.vx
        dvy = ball2.vy - ball1.vy
        vn = dvx * nx + dvy * ny
        if vn > 0:
            return  # Balls are moving apart
        # Calculate impulse
        j = -2 * vn / (1 + 1)  # Assuming equal mass for simplicity
        ball1.vx += j * nx
        ball1.vy += j * ny
        ball2.vx -= j * nx
        ball2.vy -= j * ny
        # Move balls apart slightly to avoid sticking
        overlap = 2 * ball1.radius - distance
        ball1.x -= overlap * 0.5 * nx
        ball1.y -= overlap * 0.5 * ny
        ball2.x += overlap * 0.5 * nx
        ball2.y += overlap * 0.5 * ny

class BouncingBalls:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in a Spinning Heptagon")
        self.canvas = tk.Canvas(root, width=800, height=800)
        self.canvas.pack()

        self.heptagon_radius = 300
        self.center_x = 400
        self.center_y = 400
        self.heptagon_rotation = 0
        self.rotation_speed = 360 / 5  # degrees per second

        self.balls = []
        colors = [
            "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d",
            "#ec6800", "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101",
            "#e49e61", "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
        ]
        for i in range(1, 21):
            ball = Ball(
                radius=10,
                number=i,
                color=colors[i - 1],
                x=self.center_x,
                y=self.center_y,
                vx=0,
                vy=0,
                rotation=0,
                rotation_speed=5  # arbitrary rotation speed
            )
            self.balls.append(ball)

        # Apply initial downward velocity
        for ball in self.balls:
            ball.vy = 5

        self.running = True
        self.update()

    def update(self):
        if not self.running:
            return

        # Update heptagon rotation
        self.heptagon_rotation = (self.heptagon_rotation + self.rotation_speed * 0.02) % 360

        # Update ball positions
        for ball in self.balls:
            # Apply gravity
            ball.vy += 0.1  # gravity

            # Update position
            ball.x += ball.vx
            ball.y += ball.vy

            # Check collisions with heptagon walls
            heptagon_wall_collision(ball, self.heptagon_radius, self.center_x, self.center_y, self.heptagon_rotation)

            # Check collisions with other balls
            for other in self.balls:
                if ball != other:
                    ball_collision(ball, other)

            # Update ball rotation (for visual effect)
            ball.rotation = (ball.rotation + ball.rotation_speed * 0.02) % 360

        # Render
        self.canvas.delete("all")
        # Draw heptagon
        heptagon_points = []
        for i in range(7):
            angle = math.radians(self.heptagon_rotation + i * 360 / 7)
            x = self.center_x + self.heptagon_radius * math.cos(angle)
            y = self.center_y + self.heptagon_radius * math.sin(angle)
            heptagon_points.append((x, y))
        self.canvas.create_polygon(heptagon_points, outline="black", fill="", width=2)

        # Draw balls
        for ball in self.balls:
            self.canvas.create_oval(
                ball.x - ball.radius, ball.y - ball.radius,
                ball.x + ball.radius, ball.y + ball.radius,
                fill=ball.color
            )
            self.canvas.create_text(
                ball.x, ball.y, text=str(ball.number), fill="black"
            )

        self.root.after(20, self.update)

    def stop(self):
        self.running = False

if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBalls(root)
    root.protocol("WM_DELETE_WINDOW", app.stop)  # Stop the simulation on window close
    root.mainloop()