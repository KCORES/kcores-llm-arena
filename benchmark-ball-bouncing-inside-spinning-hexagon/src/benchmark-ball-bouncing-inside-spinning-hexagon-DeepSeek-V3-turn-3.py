import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
WIDTH, HEIGHT = 800, 800
GRAVITY = 0.1
FRICTION = 0.99
BALL_RADIUS = 20
HEPTAGON_RADIUS = 350
ROTATION_SPEED = 72  # degrees per second
COLORS = ['#f8b862', '#f6ad49', '#f39800', '#f08300', '#ec6d51', '#ee7948', 
          '#ed6d3d', '#ec6800', '#ec6800', '#ee7800', '#eb6238', '#ea5506',
          '#ea5506', '#eb6101', '#e49e61', '#e45e32', '#e17b34', '#dd7a56',
          '#db8449', '#d66a35']

@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float = 0
    vy: float = 0
    angle: float = 0
    angular_velocity: float = 0

class BouncingBallsApp:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg='white')
        self.canvas.pack()
        
        self.heptagon_angle = 0
        self.balls = self.create_balls()
        self.create_heptagon()
        
        self.last_time = 0
        self.root.after(16, self.update)

    def create_heptagon(self):
        self.heptagon_points = []
        center_x, center_y = WIDTH/2, HEIGHT/2
        for i in range(7):
            angle = math.radians(360/7 * i)
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            self.heptagon_points.append((x, y))

    def create_balls(self):
        balls = []
        center_x, center_y = WIDTH/2, HEIGHT/2
        for i in range(20):
            ball = Ball(id=i+1, x=center_x, y=center_y)
            balls.append(ball)
        return balls

    def update_physics(self, dt):
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY * dt
            
            # Update position
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt
            
            # Update rotation
            ball.angle += ball.angular_velocity * dt
            ball.angular_velocity *= FRICTION

    def handle_collisions(self):
        # Handle ball-heptagon collisions
        for ball in self.balls:
            # ... collision detection and response code ...

        # Handle ball-ball collisions
        for i in range(len(self.balls)):
            for j in range(i+1, len(self.balls)):
                # ... collision detection and response code ...

    def rotate_heptagon(self, dt):
        self.heptagon_angle += ROTATION_SPEED * dt
        if self.heptagon_angle >= 360:
            self.heptagon_angle -= 360

    def draw(self):
        self.canvas.delete('all')
        
        # Draw heptagon
        rotated_points = []
        center_x, center_y = WIDTH/2, HEIGHT/2
        for x, y in self.heptagon_points:
            dx, dy = x - center_x, y - center_y
            angle = math.radians(self.heptagon_angle)
            new_x = center_x + dx * math.cos(angle) - dy * math.sin(angle)
            new_y = center_y + dx * math.sin(angle) + dy * math.cos(angle)
            rotated_points.append((new_x, new_y))
        self.canvas.create_polygon(rotated_points, outline='black', fill='')
        
        # Draw balls
        for ball in self.balls:
            x, y = ball.x, ball.y
            color = COLORS[ball.id % len(COLORS)]
            self.canvas.create_oval(x-BALL_RADIUS, y-BALL_RADIUS,
                                  x+BALL_RADIUS, y+BALL_RADIUS,
                                  fill=color)
            # Draw number
            angle = math.radians(ball.angle)
            num_x = x + BALL_RADIUS/2 * math.cos(angle)
            num_y = y + BALL_RADIUS/2 * math.sin(angle)
            self.canvas.create_text(num_x, num_y, text=str(ball.id),
                                  font=('Arial', int(BALL_RADIUS/2)))

    def update(self):
        current_time = self.root.after_idle(lambda: None)
        dt = (current_time - self.last_time) / 1000
        self.last_time = current_time
        
        self.update_physics(dt)
        self.handle_collisions()
        self.rotate_heptagon(dt)
        self.draw()
        
        self.root.after(16, self.update)

if __name__ == '__main__':
    root = tk.Tk()
    app = BouncingBallsApp(root)
    root.mainloop()
