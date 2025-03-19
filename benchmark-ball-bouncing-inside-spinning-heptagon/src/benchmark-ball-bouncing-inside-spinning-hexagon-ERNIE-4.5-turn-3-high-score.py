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
    position: np.ndarray
    velocity: np.ndarray
    angular_velocity: float = 0.0

class HeptagonSimulation:
    def __init__(self, master):
        self.master = master
        self.width, self.height = 800, 800
        self.canvas = tk.Canvas(master, width=self.width, height=self.height, bg='white')
        self.canvas.pack()

        self.heptagon_radius = 300
        self.ball_radius = 10
        self.gravity = 0.5
        self.friction = 0.99
        self.bounce_factor = 0.8
        self.rotation_speed = 2 * math.pi / 5  # 360 degrees per 5 seconds
        self.heptagon_angle = 0

        self.colors = [
            '#f8b862', '#f6ad49', '#f39800', '#f08300', '#ec6d51', '#ee7948', '#ed6d3d',
            '#ec6800', '#ec6800', '#ee7800', '#eb6238', '#ea5506', '#ea5506', '#eb6101',
            '#e49e61', '#e45e32', '#e17b34', '#dd7a56', '#db8449', '#d66a35'
        ]

        self.balls = [
            Ball(self.ball_radius, i + 1, self.colors[i], np.array([0.0, 0.0]), np.array([0.0, 3.0]))  # Initial drop velocity
            for i in range(20)
        ]

        self.running = True
        self.update()

    def update(self):
        if not self.running:
            return

        self.canvas.delete("all")
        self.heptagon_angle += self.rotation_speed / 60  # Simulate rotation over time

        # Draw heptagon
        self.draw_heptagon()

        # Update and draw balls
        for ball in self.balls:
            self.update_ball(ball)
            self.draw_ball(ball)

        # Check ball collisions
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                self.check_ball_collision(self.balls[i], self.balls[j])

        self.master.after(16, self.update)  # ~60 FPS

    def draw_heptagon(self):
        points = []
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.heptagon_angle
            x = self.width / 2 + self.heptagon_radius * math.cos(angle)
            y = self.height / 2 + self.heptagon_radius * math.sin(angle)
            points.append((x, y))
        self.canvas.create_polygon(points, outline='black', fill='', width=2)

    def draw_ball(self, ball):
        x = self.width / 2 + ball.position[0]
        y = self.height / 2 + ball.position[1]
        self.canvas.create_oval(x - ball.radius, y - ball.radius, x + ball.radius, y + ball.radius, fill=ball.color)
        self.canvas.create_text(x, y, text=str(ball.number), fill='black')

    def update_ball(self, ball):
        # Update position
        ball.position += ball.velocity / 60  # Simulate motion over time

        # Apply gravity
        ball.velocity[1] += self.gravity / 60

        # Check collision with heptagon walls
        for i in range(7):
            angle = 2 * math.pi * i / 7 + self.heptagon_angle
            wall_normal = np.array([math.cos(angle), math.sin(angle)])
            wall_point = np.array([self.heptagon_radius * math.cos(angle), self.heptagon_radius * math.sin(angle)])

            ball_to_wall = ball.position - wall_point
            proj = np.dot(ball_to_wall, wall_normal)

            if proj > self.ball_radius:
                # Ball is outside the heptagon, correct position and reflect velocity
                overlap = proj - self.ball_radius
                ball.position -= wall_normal * overlap * 1.01  # Slight push back to avoid sticking
                ball.velocity -= 2 * np.dot(ball.velocity, wall_normal) * wall_normal * self.bounce_factor

        # Apply friction
        ball.velocity *= self.friction

        # Enforce minimum bounce height (above ball radius but below heptagon radius)
        if abs(ball.velocity[1]) < 1.0:  # Adjust threshold to control bounciness
            ball.velocity[1] = -1.0 if ball.velocity[1] < 0 else 1.0

    def check_ball_collision(self, ball1, ball2):
        distance = np.linalg.norm(ball1.position - ball2.position)
        if distance < 2 * self.ball_radius:
            # Balls are colliding, resolve collision
            normal = (ball1.position - ball2.position) / distance
            relative_velocity = ball1.velocity - ball2.velocity
            impulse = np.dot(relative_velocity, normal) / 2  # Simplified impulse calculation
            ball1.velocity -= impulse * normal
            ball2.velocity += impulse * normal

            # Separate balls slightly to avoid sticking
            overlap = 2 * self.ball_radius - distance
            ball1.position += normal * overlap / 2
            ball2.position -= normal * overlap / 2

    def stop(self):
        self.running = False

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    sim = HeptagonSimulation(root)
    root.protocol("WM_DELETE_WINDOW", sim.stop)  # Gracefully stop on window close
    root.mainloop()