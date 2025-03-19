import tkinter as tk
from math import *
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 350
BALL_RADIUS = 12
GRAVITY = 0.8
FRICTION = 0.99
ROTATION_SPEED = 360 / 5  # degrees per second
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
    vx: float = 0.0
    vy: float = 0.0
    radius: float = BALL_RADIUS
    color: str
    number: int
    rotation: float = 0.0
    mass: float = 1.0

def create_balls(num_balls: int) -> List[Ball]:
    balls = []
    cx, cy = WIDTH/2, HEIGHT/2
    for i in range(num_balls):
        angle = 2 * pi * i / num_balls
        x = cx + cos(angle) * 5  # Initial spread
        y = cy + sin(angle) * 5
        color = COLORS[i % len(COLORS)]
        balls.append(Ball(x, y, 0, 0, BALL_RADIUS, color, i+1))
    return balls

def get_heptagon_vertices(center_x: float, center_y: float, radius: float, angle: float) -> List[tuple]:
    vertices = []
    for i in range(7):
        theta = 2 * pi * i / 7 + angle
        x = center_x + radius * cos(theta)
        y = center_y + radius * sin(theta)
        vertices.append((x, y))
    return vertices

def check_wall_collision(ball: Ball, vertices: List[tuple], center_x: float, center_y: float, angle: float) -> bool:
    for i in range(7):
        p1, p2 = vertices[i], vertices[(i+1)%7]
        # Calculate line segment parameters
        line_vec = (p2[0]-p1[0], p2[1]-p1[1])
        line_len_sq = line_vec[0]**2 + line_vec[1]**2
        if line_len_sq == 0:
            continue
        
        # Find closest point on segment
        t = max(0, min(1, np.dot((ball.x - p1[0], ball.y - p1[1]), line_vec)/line_len_sq))
        closest = (p1[0] + t*line_vec[0], p1[1] + t*line_vec[1])
        
        # Calculate distance from ball to line
        dx, dy = closest[0] - ball.x, closest[1] - ball.y
        dist = sqrt(dx**2 + dy**2)
        
        if dist < ball.radius:
            # Calculate normal vector
            normal = (-line_vec[1], line_vec[0])
            normal_len = sqrt(normal[0]**2 + normal[1]**2)
            if normal_len == 0:
                continue
            normal = (normal[0]/normal_len, normal[1]/normal_len)
            
            # Check if outside heptagon
            dot = np.dot((dx, dy), normal)
            if dot < 0:
                continue
            
            # Check velocity direction
            vel_dot = ball.vx * normal[0] + ball.vy * normal[1]
            if vel_dot > 0:
                continue
            
            # Calculate wall velocity at collision point
            rel_pos = (closest[0] - center_x, closest[1] - center_y)
            wall_vel = (-sin(angle), cos(angle))  # Tangential velocity
            wall_speed = ROTATION_SPEED * pi / 180 / 60  # Convert to rad/frame
            wall_vel = (wall_vel[0] * wall_speed * np.linalg.norm(rel_pos),
                        wall_vel[1] * wall_speed * np.linalg.norm(rel_pos))
            
            # Calculate relative velocity
            rel_vel = (ball.vx - wall_vel[0], ball.vy - wall_vel[1])
            rel_dot = np.dot(rel_vel, normal)
            
            # Apply collision response
            e = 0.8  # Coefficient of restitution
            impulse = -(1 + e) * rel_dot
            new_vel = (ball.vx + impulse * normal[0],
                       ball.vy + impulse * normal[1])
            
            # Update velocity
            ball.vx, ball.vy = new_vel[0] + wall_vel[0], new_vel[1] + wall_vel[1]
            
            # Position correction
            ball.x += (ball.radius - dist) * dx / dist
            ball.y += (ball.radius - dist) * dy / dist
            
            return True
    return False

def handle_ball_collisions(balls: List[Ball]):
    for i in range(len(balls)):
        for j in range(i+1, len(balls)):
            b1, b2 = balls[i], balls[j]
            dx, dy = b2.x - b1.x, b2.y - b1.y
            dist = sqrt(dx**2 + dy**2)
            if dist < 2*BALL_RADIUS:
                # Calculate collision normal and tangent
                nx, ny = dx/dist, dy/dist
                tx, ty = -ny, nx
                
                # Relative velocity
                v1n = b1.vx * nx + b1.vy * ny
                v2n = b2.vx * nx + b2.vy * ny
                
                # If already separating
                if v2n - v1n > 0:
                    continue
                
                # Calculate impulse
                e = 0.8
                impulse = (2 * (v2n - v1n)) * e / (b1.mass + b2.mass)
                
                # Apply impulse
                b1.vx -= impulse * nx * b2.mass / (b1.mass + b2.mass)
                b1.vy -= impulse * ny * b2.mass / (b1.mass + b2.mass)
                b2.vx += impulse * nx * b1.mass / (b1.mass + b2.mass)
                b2.vy += impulse * ny * b1.mass / (b1.mass + b2.mass)
                
                # Position correction
                overlap = 2*BALL_RADIUS - dist
                b1.x -= overlap * nx * b2.mass / (b1.mass + b2.mass)
                b1.y -= overlap * ny * b2.mass / (b1.mass + b2.mass)
                b2.x += overlap * nx * b1.mass / (b1.mass + b2.mass)
                b2.y += overlap * ny * b1.mass / (b1.mass + b2.mass)

def update(balls: List[Ball], angle: float, canvas: tk.Canvas):
    # Update heptagon angle
    angle += radians(ROTATION_SPEED) * (1/60)  # Assuming 60fps
    angle %= 2*pi
    
    # Get current vertices
    vertices = get_heptagon_vertices(WIDTH/2, HEIGHT/2, HEPTAGON_RADIUS, angle)
    
    # Update balls
    for ball in balls:
        # Apply gravity and friction
        ball.vy += GRAVITY
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        
        # Check wall collisions
        if not check_wall_collision(ball, vertices, WIDTH/2, HEIGHT/2, angle):
            # Update position if no collision
            ball.x += ball.vx
            ball.y += ball.vy
        
        # Update rotation
        ball.rotation += (abs(ball.vx) + abs(ball.vy)) * 0.1
        ball.rotation %= 2*pi
    
    # Handle ball collisions
    handle_ball_collisions(balls)
    
    # Redraw
    canvas.delete("all")
    canvas.create_polygon(vertices, outline="black", fill="", width=2)
    for ball in balls:
        x0 = ball.x - ball.radius
        y0 = ball.y - ball.radius
        x1 = x0 + 2*ball.radius
        y1 = y0 + 2*ball.radius
        canvas.create_oval(x0, y0, x1, y1, fill=ball.color, outline="")
        canvas.create_text(ball.x, ball.y, text=str(ball.number),
                          fill="black", font=("Arial", 10, "bold"),
                          angle=degrees(ball.rotation))
    
    canvas.after(16, update, balls, angle, canvas)

def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()
    
    balls = create_balls(20)
    update(balls, 0.0, canvas)
    
    root.mainloop()

if __name__ == "__main__":
    main()