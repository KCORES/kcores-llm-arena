import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple

# Constants
CANVAS_WIDTH = 800
CANVAS_HEIGHT = 800
HEPTAGON_RADIUS = 350
BALL_RADIUS = 15
GRAVITY = 0.5
FRICTION = 0.98
BOUNCE_FACTOR = 0.85
SPIN_SPEED = 360 / 5  # degrees per second
BALL_COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", 
    "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800", 
    "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", 
    "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
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
    canvas_id: int
    text_id: int

class BouncingBalls:
    def __init__(self, root):
        self.root = root
        self.root.title("Bouncing Balls in Spinning Heptagon")
        self.canvas = tk.Canvas(root, width=CANVAS_WIDTH, height=CANVAS_HEIGHT, bg="white")
        self.canvas.pack()
        
        # Initialize heptagon
        self.heptagon_angle = 0
        self.heptagon_center = (CANVAS_WIDTH // 2, CANVAS_HEIGHT // 2)
        self.heptagon_points = self.calculate_heptagon_points()
        
        # Initialize balls
        self.balls = []
        self.initialize_balls()
        
        # Animation variables
        self.last_time = 0
        self.animation_id = None
        
        # Start animation
        self.animate()
    
    def calculate_heptagon_points(self):
        center_x, center_y = self.heptagon_center
        points = []
        for i in range(7):
            angle = math.radians(self.heptagon_angle + i * 360/7)
            x = center_x + HEPTAGON_RADIUS * math.cos(angle)
            y = center_y + HEPTAGON_RADIUS * math.sin(angle)
            points.append((x, y))
        return points
    
    def initialize_balls(self):
        center_x, center_y = self.heptagon_center
        
        for i in range(20):
            # Start all balls at center with random initial velocities
            x = center_x
            y = center_y
            vx = np.random.uniform(-2, 2)
            vy = np.random.uniform(-2, 2)
            angular_vel = np.random.uniform(-0.1, 0.1)
            
            # Create ball and number text
            ball_id = self.canvas.create_oval(
                x - BALL_RADIUS, y - BALL_RADIUS,
                x + BALL_RADIUS, y + BALL_RADIUS,
                fill=BALL_COLORS[i], outline=""
            )
            
            text_id = self.canvas.create_text(
                x, y, text=str(i+1), font=("Arial", 12, "bold"),
                fill="black"
            )
            
            self.balls.append(Ball(
                x=x, y=y, vx=vx, vy=vy, angular_vel=angular_vel,
                color=BALL_COLORS[i], number=i+1,
                canvas_id=ball_id, text_id=text_id
            ))
    
    def update_heptagon(self, dt):
        # Update heptagon rotation
        self.heptagon_angle += math.radians(SPIN_SPEED * dt / 1000)
        self.heptagon_points = self.calculate_heptagon_points()
        
        # Draw heptagon
        self.canvas.delete("heptagon")
        self.canvas.create_polygon(
            self.heptagon_points, outline="black", fill="", width=2, tags="heptagon"
        )
    
    def update_balls(self, dt):
        # Update ball positions with physics
        for ball in self.balls:
            # Apply gravity
            ball.vy += GRAVITY * dt / 1000
            
            # Update position
            ball.x += ball.vx * dt / 1000
            ball.y += ball.vy * dt / 1000
            
            # Update rotation
            ball.angular_vel *= FRICTION ** (dt / 1000)
            ball.angular_vel += np.random.uniform(-0.001, 0.001)
            
            # Update ball and text position
            self.canvas.coords(ball.canvas_id, 
                             ball.x - BALL_RADIUS, ball.y - BALL_RADIUS,
                             ball.x + BALL_RADIUS, ball.y + BALL_RADIUS)
            self.canvas.coords(ball.text_id, ball.x, ball.y)
            
            # Rotate text based on angular velocity
            angle = math.degrees(ball.angular_vel * dt / 1000)
            self.canvas.itemconfig(ball.text_id, angle=angle)
    
    def check_ball_collisions(self):
        # Check collisions between balls
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                
                dx = ball2.x - ball1.x
                dy = ball2.y - ball1.y
                dist_sq = dx*dx + dy*dy
                min_dist_sq = (2 * BALL_RADIUS) ** 2
                
                if dist_sq < min_dist_sq:
                    # Calculate collision normal
                    dist = math.sqrt(dist_sq)
                    nx = dx / dist
                    ny = dy / dist
                    
                    # Calculate relative velocity
                    dvx = ball2.vx - ball1.vx
                    dvy = ball2.vy - ball1.vy
                    
                    # Calculate impulse scalar
                    j = (-(dvx * nx + dvy * ny) * (1 + BOUNCE_FACTOR)) / (2)
                    
                    # Apply impulse
                    impulse_x = j * nx
                    impulse_y = j * ny
                    
                    ball1.vx += impulse_x
                    ball1.vy += impulse_y
                    ball2.vx -= impulse_x
                    ball2.vy -= impulse_y
                    
                    # Move balls apart
                    overlap = 2 * BALL_RADIUS - dist
                    ball1.x -= overlap * 0.5 * nx
                    ball1.y -= overlap * 0.5 * ny
                    ball2.x += overlap * 0.5 * nx
                    ball2.y += overlap * 0.5 * ny
    
    def check_wall_collisions(self):
        # Check collisions with heptagon walls
        for ball in self.balls:
            # Find closest point on heptagon edges
            closest_x, closest_y, normal_x, normal_y = self.find_closest_point(ball.x, ball.y)
            
            dx = ball.x - closest_x
            dy = ball.y - closest_y
            dist_sq = dx*dx + dy*dy
            
            if dist_sq < BALL_RADIUS**2:
                # Collision detected
                # Calculate penetration depth
                penetration = BALL_RADIUS - math.sqrt(dist_sq)
                
                # Move ball out of wall
                ball.x -= penetration * normal_x
                ball.y -= penetration * normal_y
                
                # Reflect velocity with bounce factor and friction
                dot_product = ball.vx * normal_x + ball.vy * normal_y
                ball.vx -= 2 * dot_product * normal_x * BOUNCE_FACTOR
                ball.vy -= 2 * dot_product * normal_y * BOUNCE_FACTOR
                
                # Apply friction
                tangent_x = -normal_y
                tangent_y = normal_x
                tangent_speed = ball.vx * tangent_x + ball.vy * tangent_y
                ball.vx -= tangent_speed * tangent_x * FRICTION
                ball.vy -= tangent_speed * tangent_y * FRICTION
    
    def find_closest_point(self, x, y):
        min_dist_sq = float('inf')
        closest_x, closest_y = 0, 0
        normal_x, normal_y = 0, 0
        
        # Check each edge of the heptagon
        for i in range(7):
            p1 = self.heptagon_points[i]
            p2 = self.heptagon_points[(i + 1) % 7]
            
            # Vector from p1 to p2
            edge_x = p2[0] - p1[0]
            edge_y = p2[1] - p1[1]
            
            # Vector from p1 to ball
            to_ball_x = x - p1[0]
            to_ball_y = y - p1[1]
            
            # Project to_ball onto edge
            dot_product = to_ball_x * edge_x + to_ball_y * edge_y
            edge_length_sq = edge_x*edge_x + edge_y*edge_y
            
            # Find closest point on edge
            t = max(0, min(1, dot_product / edge_length_sq))
            closest_edge_x = p1[0] + t * edge_x
            closest_edge_y = p1[1] + t * edge_y
            
            # Calculate distance squared
            dx = x - closest_edge_x
            dy = y - closest_edge_y
            dist_sq = dx*dx + dy*dy
            
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_x, closest_y = closest_edge_x, closest_edge_y
                
                # Calculate normal vector (perpendicular to edge)
                normal_x = -edge_y
                normal_y = edge_x
                norm = math.sqrt(normal_x*normal_x + normal_y*normal_y)
                if norm != 0:
                    normal_x /= norm
                    normal_y /= norm
        
        return closest_x, closest_y, normal_x, normal_y
    
    def animate(self):
        current_time = self.root.after_idle(self.animate)
        if self.last_time == 0:
            self.last_time = current_time
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Update heptagon
        self.update_heptagon(dt)
        
        # Update balls
        self.update_balls(dt)
        
        # Check collisions
        self.check_ball_collisions()
        self.check_wall_collisions()
        
        # Continue animation
        self.animation_id = self.root.after(10, self.animate)

if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBalls(root)
    root.mainloop()