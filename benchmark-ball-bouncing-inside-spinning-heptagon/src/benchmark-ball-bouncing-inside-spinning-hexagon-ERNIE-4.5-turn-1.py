import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class Ball:
    radius: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    color: str
    number: int
    rotation: float = 0.0  # rotation in degrees for the number display

@dataclass
class Heptagon:
    center_x: float
    center_y: float
    radius: float
    rotation: float = 0.0  # rotation in degrees
    rotation_speed: float = 72.0  # degrees per second (360 degrees in 5 seconds)

def rotate_point(x, y, cx, cy, angle_deg):
    angle_rad = math.radians(angle_deg)
    dx = x - cx
    dy = y - cy
    new_x = cx + dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
    new_y = cy + dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
    return new_x, new_y

def check_wall_collision(ball, heptagon, dt):
    angle_step = 360 / 7
    for i in range(7):
        angle = math.radians(heptagon.rotation + i * angle_step)
        wall_x = heptagon.center_x + heptagon.radius * math.cos(angle)
        wall_y = heptagon.center_y + heptagon.radius * math.sin(angle)
        wall_normal_x = -math.sin(angle)
        wall_normal_y = math.cos(angle)
        
        # Project ball's position to the wall line
        dx = ball.x - wall_x
        dy = ball.y - wall_y
        projection = dx * wall_normal_x + dy * wall_normal_y
        
        if abs(projection) <= ball.radius:  # Ball is close enough to check for collision
            # Check if ball is within the line segment's bounds (approximated for simplicity)
            wall_to_ball_vec = np.array([ball.x - wall_x, ball.y - wall_y])
            wall_tangent = np.array([wall_normal_y, -wall_normal_x])  # Perpendicular to normal
            if np.linalg.norm(wall_to_ball_vec) <= heptagon.radius:  # Simplistic segment check
                # Reflect velocity
                dot = ball.vx * wall_normal_x + ball.vy * wall_normal_y
                ball.vx -= 2 * dot * wall_normal_x
                ball.vy -= 2 * dot * wall_normal_y
                
                # Apply some damping (friction-like effect)
                ball.vx *= 0.95
                ball.vy *= 0.95

def check_ball_collisions(balls):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            dx = balls[j].x - balls[i].x
            dy = balls[j].y - balls[i].y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < 2 * balls[i].radius:  # Collision detected
                # Normalize the distance vector
                nx = dx / distance
                ny = dy / distance
                
                # Project velocities onto the normal
                dvx = balls[j].vx - balls[i].vx
                dvy = balls[j].vy - balls[i].vy
                vn = dvx * nx + dvy * ny
                
                if vn > 0:  # Balls are moving apart, no collision response needed
                    continue
                
                # Apply impulse to separate balls (very simplistic)
                impulse = -0.5 * vn  # Arbitrary coefficient for impulse magnitude
                balls[i].vx += impulse * nx
                balls[i].vy += impulse * ny
                balls[j].vx -= impulse * nx
                balls[j].vy -= impulse * ny

def update_simulation(balls, heptagon, dt, canvas_width, canvas_height):
    gravity = 0.1
    friction = 0.99
    
    for ball in balls:
        # Update velocity with gravity
        ball.vy += gravity * dt
        
        # Update position
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        
        # Apply friction to velocity (damping)
        ball.vx *= friction
        ball.vy *= friction
        
        # Bounce off heptagon walls
        check_wall_collision(ball, heptagon, dt)
    
    # Check for collisions between balls
    check_ball_collisions(balls)
    
    # Update heptagon rotation
    heptagon.rotation += heptagon.rotation_speed * dt
    heptagon.rotation %= 360

def draw_simulation(canvas, balls, heptagon):
    canvas.delete("all")
    
    # Draw heptagon
    heptagon_points = []
    angle_step = 360 / 7
    for i in range(7):
        angle = math.radians(heptagon.rotation + i * angle_step)
        x = heptagon.center_x + heptagon.radius * math.cos(angle)
        y = heptagon.center_y + heptagon.radius * math.sin(angle)
        heptagon_points.append((x, y))
    canvas.create_polygon(
        [x for x, y in heptagon_points] + [heptagon_points[0][0]],
        [y for x, y in heptagon_points] + [heptagon_points[0][1]],
        outline="black", fill=""
    )
    
    # Draw balls
    for ball in balls:
        # Draw ball
        canvas.create_oval(
            ball.x - ball.radius, ball.y - ball.radius,
            ball.x + ball.radius, ball.y + ball.radius,
            fill=ball.color, outline="black"
        )
        # Draw number
        canvas.create_text(
            ball.x, ball.y, text=str(ball.number), fill="white"
        )

def main():
    # Setup tkinter window
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    canvas_width, canvas_height = 800, 800
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
    canvas.pack()
    
    # Heptagon parameters
    heptagon = Heptagon(center_x=canvas_width / 2, center_y=canvas_height / 2, radius=300)
    
    # Ball parameters
    ball_radius = 10
    ball_colors = [
        "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d", "#ec6800",
        "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", "#e45e32",
        "#e17b34", "#dd7a56", "#db8449", "#d66a35"
    ]
    balls = [
        Ball(radius=ball_radius, x=heptagon.center_x, y=heptagon.center_y, color=ball_colors[i], number=i + 1)
        for i in range(20)
    ]
    
    # Simulation loop
    dt = 0.016  # Approximately 60 FPS
    def loop():
        update_simulation(balls, heptagon, dt, canvas_width, canvas_height)
        draw_simulation(canvas, balls, heptagon)
        root.after(int(dt * 1000), loop)
    
    loop()
    root.mainloop()

if __name__ == "__main__":
    main()