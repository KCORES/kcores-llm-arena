import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, List

# Constants
CANVAS_SIZE = 800
HEPTAGON_RADIUS = 350
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.98
BOUNCE_FACTOR = 0.8
ROTATION_SPEED = 2 * math.pi / 5  # 360 degrees in 5 seconds
FPS = 60
TIME_STEP = 1000 / FPS

# Colors for balls
BALL_COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d",
    "#ec6800", "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101",
    "#e49e61", "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]

@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    angular_vel: float
    color: str
    number: int

class BouncingBalls:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg='white')
        self.canvas.pack()
        
        self.angle = 0
        self.balls = []
        self.init_balls()
        
    def init_balls(self):
        center = CANVAS_SIZE // 2
        for i in range(20):
            self.balls.append(Ball(
                x=center,
                y=center,
                vx=np.random.uniform(-50, 50),
                vy=np.random.uniform(-50, 50),
                angular_vel=np.random.uniform(-0.1, 0.1),
                color=BALL_COLORS[i],
                number=i+1
            ))
            
    def get_heptagon_vertices(self) -> List[Tuple[float, float]]:
        vertices = []
        center = CANVAS_SIZE // 2
        for i in range(7):
            angle = self.angle + i * 2 * math.pi / 7
            x = center + HEPTAGON_RADIUS * math.cos(angle)
            y = center + HEPTAGON_RADIUS * math.sin(angle)
            vertices.append((x, y))
        return vertices
    
    def check_ball_wall_collision(self, ball: Ball, vertices: List[Tuple[float, float]]):
        for i in range(len(vertices)):
            p1 = vertices[i]
            p2 = vertices[(i+1) % len(vertices)]
            
            # Vector from p1 to p2
            edge_vec = (p2[0] - p1[0], p2[1] - p1[1])
            edge_len = math.hypot(edge_vec[0], edge_vec[1])
            edge_unit = (edge_vec[0]/edge_len, edge_vec[1]/edge_len)
            
            # Vector from p1 to ball
            to_ball = (ball.x - p1[0], ball.y - p1[1])
            
            # Project to_ball onto edge_vec
            proj_len = to_ball[0]*edge_unit[0] + to_ball[1]*edge_unit[1]
            proj_len = max(0, min(edge_len, proj_len))
            
            # Closest point on edge
            closest_x = p1[0] + edge_unit[0] * proj_len
            closest_y = p1[1] + edge_unit[1] * proj_len
            
            # Distance from ball to edge
            distance = math.hypot(ball.x - closest_x, ball.y - closest_y)
            
            if distance <= BALL_RADIUS:
                # Calculate normal vector
                normal = (ball.x - closest_x, ball.y - closest_y)
                normal_len = math.hypot(normal[0], normal[1])
                if normal_len > 0:
                    normal = (normal[0]/normal_len, normal[1]/normal_len)
                    
                    # Calculate reflection
                    dot_product = ball.vx * normal[0] + ball.vy * normal[1]
                    ball.vx -= 2 * dot_product * normal[0] * BOUNCE_FACTOR
                    ball.vy -= 2 * dot_product * normal[1] * BOUNCE_FACTOR
                    
                    # Apply friction
                    tangent = (-normal[1], normal[0])
                    tangential_speed = ball.vx * tangent[0] + ball.vy * tangent[1]
                    ball.vx -= tangent[0] * tangential_speed * (1 - FRICTION)
                    ball.vy -= tangent[1] * tangential_speed * (1 - FRICTION)
                    
                    # Move ball outside collision
                    overlap = BALL_RADIUS - distance
                    ball.x += normal[0] * overlap
                    ball.y += normal[1] * overlap
    
    def check_ball_ball_collision(self, balls: List[Ball]):
        for i in range(len(balls)):
            for j in range(i+1, len(balls)):
                dx = balls[i].x - balls[j].x
                dy = balls[i].y - balls[j].y
                distance_sq = dx*dx + dy*dy
                min_dist = 2 * BALL_RADIUS
                
                if distance_sq < min_dist**2:
                    distance = math.sqrt(distance_sq)
                    overlap = (min_dist - distance) / 2
                    
                    # Calculate normal
                    normal_x = dx / distance
                    normal_y = dy / distance
                    
                    # Move balls apart
                    balls[i].x += normal_x * overlap
                    balls[i].y += normal_y * overlap
                    balls[j].x -= normal_x * overlap
                    balls[j].y -= normal_y * overlap
                    
                    # Calculate velocities
                    vel_i = np.array([balls[i].vx, balls[i].vy])
                    vel_j = np.array([balls[j].vx, balls[j].vy])
                    normal = np.array([normal_x, normal_y])
                    
                    # Elastic collision response
                    dot_product = np.dot(vel_i - vel_j, normal)
                    balls[i].vx -= dot_product * normal[0]
                    balls[i].vy -= dot_product * normal[1]
                    balls[j].vx += dot_product * normal[0]
                    balls[j].vy += dot_product * normal[1]
    
    def update(self):
        # Update heptagon rotation
        self.angle += ROTATION_SPEED * TIME_STEP / 1000
        vertices = self.get_heptagon_vertices()
        
        # Update ball physics
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY * TIME_STEP / 1000
            
            # Apply friction to rotation
            ball.angular_vel *= FRICTION
            
            # Update position
            ball.x += ball.vx * TIME_STEP / 1000
            ball.y += ball.vy * TIME_STEP / 1000
            
            # Check wall collisions
            self.check_ball_wall_collision(ball, vertices)
        
        # Check ball-ball collisions
        self.check_ball_ball_collision(self.balls)
        
        # Redraw
        self.canvas.delete("all")
        self.canvas.create_polygon(vertices, outline='black', fill='', width=2)
        
        for ball in self.balls:
            x1 = ball.x - BALL_RADIUS
            y1 = ball.y - BALL_RADIUS
            x2 = ball.x + BALL_RADIUS
            y2 = ball.y + BALL_RADIUS
            self.canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline='black')
            
            # Draw number
            self.canvas.create_text(
                ball.x, ball.y,
                text=str(ball.number),
                font=('Arial', 12, 'bold'),
                fill='black'
            )
        
        self.root.after(int(TIME_STEP), self.update)
    
    def run(self):
        self.update()
        self.root.mainloop()

if __name__ == "__main__":
    simulation = BouncingBalls()
    simulation.run()