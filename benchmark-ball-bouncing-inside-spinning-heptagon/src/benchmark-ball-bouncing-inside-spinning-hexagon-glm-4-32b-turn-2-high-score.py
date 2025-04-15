import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import time

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
GRAVITY = 9.8 * 100  # Scaled for pixel space
FRICTION_COEFFICIENT = 0.7
RESTITUTION_COEFFICIENT = 0.8  # Bounce coefficient
ANGULAR_FRICTION = 0.98  # Rotational friction
HEPTAGON_ROTATION_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Distance from center to vertices

# Colors for the balls
BALL_COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", 
    "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800", 
    "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", 
    "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]

@dataclass
class Vector2D:
    x: float
    y: float
    
    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar):
        return Vector2D(self.x * scalar, self.y * scalar)
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y
    
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def normalize(self):
        length = self.length()
        if length > 0:
            return Vector2D(self.x / length, self.y / length)
        return Vector2D(0, 0)
    
    def rotate(self, angle):
        """Rotate vector by angle (in radians)"""
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        return Vector2D(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a
        )

class Ball:
    def __init__(self, number, color, position, velocity=None, angular_velocity=0):
        self.number = number
        self.color = color
        self.position = position
        self.velocity = velocity if velocity else Vector2D(0, 0)
        self.angular_velocity = angular_velocity  # Radians per second
        self.rotation = 0  # Current rotation in radians
        self.mass = 1.0
        self.moment_of_inertia = 0.5 * self.mass * BALL_RADIUS**2  # For a solid disk
    
    def update(self, dt):
        # Update position based on velocity
        self.position = self.position + self.velocity * dt
        
        # Apply gravity
        self.velocity.y += GRAVITY * dt
        
        # Update rotation based on angular velocity
        self.rotation += self.angular_velocity * dt
        
        # Apply angular friction
        self.angular_velocity *= ANGULAR_FRICTION

class Heptagon:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.rotation = 0  # Current rotation in radians
        self.vertices = self._calculate_vertices()
        self.edges = self._calculate_edges()
    
    def _calculate_vertices(self):
        vertices = []
        for i in range(7):
            angle = self.rotation + i * 2 * math.pi / 7
            x = self.center.x + self.radius * math.cos(angle)
            y = self.center.y + self.radius * math.sin(angle)
            vertices.append(Vector2D(x, y))
        return vertices
    
    def _calculate_edges(self):
        edges = []
        for i in range(7):
            edges.append((self.vertices[i], self.vertices[(i + 1) % 7]))
        return edges
    
    def update(self, dt):
        self.rotation += HEPTAGON_ROTATION_SPEED * dt
        self.vertices = self._calculate_vertices()
        self.edges = self._calculate_edges()

class PhysicsSimulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="black")
        self.canvas.pack()
        
        # Create heptagon
        center = Vector2D(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
        self.heptagon = Heptagon(center, HEPTAGON_RADIUS)
        
        # Create balls
        self.balls = []
        for i in range(20):
            # All balls start at the center with small random velocities
            position = Vector2D(center.x, center.y)
            velocity = Vector2D(
                (np.random.random() - 0.5) * 50,  # Random x velocity
                (np.random.random() - 0.5) * 50   # Random y velocity
            )
            angular_velocity = (np.random.random() - 0.5) * 5  # Random spin
            ball = Ball(i + 1, BALL_COLORS[i], position, velocity, angular_velocity)
            self.balls.append(ball)
        
        self.last_time = time.time()
        self.update()
    
    def update(self):
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.05)  # Cap dt to avoid large jumps
        self.last_time = current_time
        
        # Update heptagon
        self.heptagon.update(dt)
        
        # Update balls
        for ball in self.balls:
            ball.update(dt)
        
        # Check for collisions
        self.check_ball_wall_collisions()
        self.check_ball_ball_collisions()
        
        # Redraw everything
        self.draw()
        
        # Schedule next update
        self.root.after(16, self.update)  # ~60 FPS
    
    def check_ball_wall_collisions(self):
        for ball in self.balls:
            # Check collision with each edge of the heptagon
            for edge in self.heptagon.edges:
                start, end = edge
                
                # Vector from start to end
                edge_vector = end - start
                edge_length = edge_vector.length()
                edge_normal = Vector2D(-edge_vector.y / edge_length, edge_vector.x / edge_length)
                
                # Vector from start to ball
                to_ball = ball.position - start
                
                # Project to_ball onto edge_vector
                projection_length = to_ball.dot(edge_vector) / edge_length
                
                # Find closest point on edge to ball
                if projection_length < 0:
                    closest_point = start
                elif projection_length > edge_length:
                    closest_point = end
                else:
                    closest_point = start + edge_vector.normalize() * projection_length
                
                # Check if ball is colliding with edge
                distance = (ball.position - closest_point).length()
                if distance < BALL_RADIUS:
                    # Calculate penetration depth
                    penetration = BALL_RADIUS - distance
                    
                    # Move ball out of collision
                    correction = (ball.position - closest_point).normalize() * penetration
                    ball.position = ball.position + correction
                    
                    # Calculate reflection
                    normal = (ball.position - closest_point).normalize()
                    
                    # Calculate relative velocity
                    relative_velocity = ball.velocity
                    
                    # Calculate impulse
                    impulse = normal.dot(relative_velocity) * -(1 + RESTITUTION_COEFFICIENT)
                    
                    # Apply impulse to velocity
                    ball.velocity = ball.velocity + normal * impulse
                    
                    # Apply friction to tangential component
                    tangent = Vector2D(-normal.y, normal.x)
                    tangential_velocity = tangent.dot(ball.velocity)
                    
                    # Apply friction to tangential velocity
                    friction_impulse = -tangential_velocity * FRICTION_COEFFICIENT
                    ball.velocity = ball.velocity + tangent * friction_impulse
                    
                    # Apply angular velocity change based on friction and tangential velocity
                    ball.angular_velocity += friction_impulse * BALL_RADIUS / ball.moment_of_inertia
    
    def check_ball_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                
                # Vector from ball1 to ball2
                delta = ball2.position - ball1.position
                distance = delta.length()
                
                # Check if balls are colliding
                if distance < 2 * BALL_RADIUS:
                    # Calculate penetration depth
                    penetration = 2 * BALL_RADIUS - distance
                    
                    # Normal vector from ball1 to ball2
                    normal = delta.normalize() if distance > 0 else Vector2D(1, 0)
                    
                    # Move balls apart to resolve penetration
                    correction = normal * (penetration / 2)
                    ball1.position = ball1.position - correction
                    ball2.position = ball2.position + correction
                    
                    # Calculate relative velocity
                    relative_velocity = ball2.velocity - ball1.velocity
                    
                    # Calculate impulse
                    impulse_magnitude = normal.dot(relative_velocity) * -(1 + RESTITUTION_COEFFICIENT)
                    impulse_magnitude /= (1 / ball1.mass + 1 / ball2.mass)
                    
                    # Apply impulse to velocities
                    ball1.velocity = ball1.velocity - normal * (impulse_magnitude / ball1.mass)
                    ball2.velocity = ball2.velocity + normal * (impulse_magnitude / ball2.mass)
                    
                    # Calculate tangential component for friction
                    tangent = Vector2D(-normal.y, normal.x)
                    tangential_velocity = tangent.dot(relative_velocity)
                    
                    # Apply friction to tangential velocity
                    friction_impulse = -tangential_velocity * FRICTION_COEFFICIENT
                    friction_impulse /= (1 / ball1.mass + 1 / ball2.mass)
                    
                    ball1.velocity = ball1.velocity - tangent * (friction_impulse / ball1.mass)
                    ball2.velocity = ball2.velocity + tangent * (friction_impulse / ball2.mass)
                    
                    # Apply angular velocity changes based on friction and tangential velocity
                    ball1.angular_velocity -= friction_impulse * BALL_RADIUS / ball1.moment_of_inertia
                    ball2.angular_velocity += friction_impulse * BALL_RADIUS / ball2.moment_of_inertia
    
    def draw(self):
        self.canvas.delete("all")
        
        # Draw heptagon
        polygon_points = []
        for vertex in self.heptagon.vertices:
            polygon_points.extend([vertex.x, vertex.y])
        self.canvas.create_polygon(polygon_points, outline="white", fill="", width=2)
        
        # Draw balls
        for ball in self.balls:
            x, y = ball.position.x, ball.position.y
            
            # Draw the ball
            self.canvas.create_oval(
                x - BALL_RADIUS, y - BALL_RADIUS,
                x + BALL_RADIUS, y + BALL_RADIUS,
                fill=ball.color, outline="white"
            )
            
            # Draw the number with rotation to show spin
            text_x = x + 0.6 * BALL_RADIUS * math.cos(ball.rotation)
            text_y = y + 0.6 * BALL_RADIUS * math.sin(ball.rotation)
            
            self.canvas.create_text(
                text_x, text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(BALL_RADIUS * 0.8))
            )

def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    
    simulation = PhysicsSimulation(root)
    
    root.mainloop()

if __name__ == "__main__":
    main()