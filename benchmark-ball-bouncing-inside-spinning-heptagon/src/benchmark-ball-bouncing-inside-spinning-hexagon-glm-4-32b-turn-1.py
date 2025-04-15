import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import time
import random

# Constants
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800
GRAVITY = 9.8 * 100  # Scaled gravity (pixels/s^2)
FRICTION_COEFFICIENT = 0.3  # Friction coefficient for ball-wall collisions
ROLLING_FRICTION = 0.05  # Friction coefficient for ball rolling
AIR_RESISTANCE = 0.01  # Air resistance coefficient
RESTITUTION = 0.7  # Coefficient of restitution (bounciness)
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Distance from center to vertices
HEPTAGON_ROTATION_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds (in radians/second)
BALL_COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d", "#ec6800", 
    "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", "#e45e32", 
    "#e17b34", "#dd7a56", "#db8449", "#d66a35"
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
        self.angular_velocity = angular_velocity  # radians per second
        self.rotation = 0  # current rotation in radians
        self.mass = 1.0
        self.moment_of_inertia = 0.5 * self.mass * BALL_RADIUS**2  # for a solid disk
    
    def update(self, dt):
        # Update position based on velocity
        self.position = self.position + self.velocity * dt
        
        # Update rotation based on angular velocity
        self.rotation += self.angular_velocity * dt
        
        # Apply gravity
        self.velocity.y += GRAVITY * dt
        
        # Apply air resistance
        speed = self.velocity.length()
        if speed > 0:
            drag = AIR_RESISTANCE * speed * speed
            drag_vector = self.velocity.normalize() * (-drag)
            self.velocity = self.velocity + drag_vector * dt
        
        # Apply rolling friction to angular velocity
        if self.angular_velocity != 0:
            friction_torque = -ROLLING_FRICTION * self.angular_velocity
            angular_acceleration = friction_torque / self.moment_of_inertia
            self.angular_velocity += angular_acceleration * dt
            
            # Stop rotation if it's very slow
            if abs(self.angular_velocity) < 0.01:
                self.angular_velocity = 0

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
    
    def draw(self, canvas):
        points = []
        for vertex in self.vertices:
            points.extend([vertex.x, vertex.y])
        canvas.create_polygon(points, outline="black", fill="", width=2)

class PhysicsSimulation:
    def __init__(self, root):
        self.root = root
        self.canvas = tk.Canvas(root, width=WINDOW_WIDTH, height=WINDOW_HEIGHT, bg="white")
        self.canvas.pack()
        
        # Create heptagon
        center = Vector2D(WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2)
        self.heptagon = Heptagon(center, HEPTAGON_RADIUS)
        
        # Create balls
        self.balls = []
        for i in range(20):
            # All balls start at the center with random initial velocities
            position = Vector2D(center.x, center.y)
            velocity = Vector2D(
                random.uniform(-50, 50),
                random.uniform(-50, 50)
            )
            angular_velocity = random.uniform(-2, 2)  # Random initial spin
            ball = Ball(i + 1, BALL_COLORS[i], position, velocity, angular_velocity)
            self.balls.append(ball)
        
        self.last_time = time.time()
        self.update()
    
    def update(self):
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.05)  # Cap dt to avoid large time steps
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
                
                # Vector from start to ball
                to_ball = ball.position - start
                
                # Project to_ball onto edge_vector to find closest point
                t = max(0, min(1, to_ball.dot(edge_vector) / (edge_length * edge_length)))
                closest_point = start + edge_vector * t
                
                # Vector from closest point to ball
                to_ball_from_closest = ball.position - closest_point
                distance = to_ball_from_closest.length()
                
                # Check if collision occurred
                if distance <= BALL_RADIUS:
                    # Calculate normal vector (perpendicular to edge)
                    normal = Vector2D(-edge_vector.y, edge_vector.x).normalize()
                    
                    # Make sure normal points toward the ball
                    if normal.dot(to_ball_from_closest) < 0:
                        normal = normal * (-1)
                    
                    # Calculate relative velocity
                    rel_velocity = ball.velocity
                    
                    # Calculate velocity component along normal
                    normal_velocity = normal.dot(rel_velocity)
                    
                    # Only resolve collision if objects are moving toward each other
                    if normal_velocity < 0:
                        # Calculate impulse
                        impulse = -(1 + RESTITUTION) * normal_velocity
                        
                        # Apply impulse to ball's velocity
                        ball.velocity = ball.velocity + normal * impulse
                        
                        # Apply friction to tangential component
                        tangent = Vector2D(-normal.y, normal.x)
                        tangent_velocity = tangent.dot(rel_velocity)
                        
                        # Apply friction impulse
                        friction_impulse = -FRICTION_COEFFICIENT * tangent_velocity
                        ball.velocity = ball.velocity + tangent * friction_impulse
                        
                        # Apply torque to ball based on friction (causes rotation)
                        ball.angular_velocity += friction_impulse * BALL_RADIUS / ball.moment_of_inertia
                        
                        # Move ball outside of wall to prevent sticking
                        penetration_depth = BALL_RADIUS - distance
                        ball.position = ball.position + normal * penetration_depth
    
    def check_ball_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                
                # Vector from ball1 to ball2
                delta = ball2.position - ball1.position
                distance = delta.length()
                
                # Check if collision occurred
                if distance <= 2 * BALL_RADIUS:
                    # Calculate normal vector
                    normal = delta.normalize()
                    
                    # Calculate relative velocity
                    rel_velocity = ball2.velocity - ball1.velocity
                    
                    # Calculate velocity component along normal
                    normal_velocity = normal.dot(rel_velocity)
                    
                    # Only resolve collision if objects are moving toward each other
                    if normal_velocity < 0:
                        # Calculate impulse
                        impulse = -(1 + RESTITUTION) * normal_velocity / 2
                        
                        # Apply impulse to both balls
                        ball1.velocity = ball1.velocity - normal * impulse
                        ball2.velocity = ball2.velocity + normal * impulse
                        
                        # Apply torque to balls based on impact point
                        # Tangential vector at point of impact
                        tangent = Vector2D(-normal.y, normal.x)
                        
                        # Calculate tangential velocity
                        tangent_velocity = tangent.dot(rel_velocity)
                        
                        # Apply friction impulse
                        friction_impulse = -FRICTION_COEFFICIENT * tangent_velocity / 2
                        
                        ball1.velocity = ball1.velocity - tangent * friction_impulse
                        ball2.velocity = ball2.velocity + tangent * friction_impulse
                        
                        # Apply torque to balls
                        ball1.angular_velocity -= friction_impulse * BALL_RADIUS / ball1.moment_of_inertia
                        ball2.angular_velocity += friction_impulse * BALL_RADIUS / ball2.moment_of_inertia
                        
                        # Move balls apart to prevent sticking
                        penetration_depth = 2 * BALL_RADIUS - distance
                        correction = normal * (penetration_depth / 2)
                        ball1.position = ball1.position - correction
                        ball2.position = ball2.position + correction
    
    def draw(self):
        self.canvas.delete("all")
        
        # Draw heptagon
        self.heptagon.draw(self.canvas)
        
        # Draw balls
        for ball in self.balls:
            # Draw ball
            self.canvas.create_oval(
                ball.position.x - BALL_RADIUS,
                ball.position.y - BALL_RADIUS,
                ball.position.x + BALL_RADIUS,
                ball.position.y + BALL_RADIUS,
                fill=ball.color,
                outline="black"
            )
            
            # Draw number with rotation
            text_x = ball.position.x
            text_y = ball.position.y
            
            # Create text with rotation
            self.canvas.create_text(
                text_x, text_y,
                text=str(ball.number),
                fill="white",
                font=("Arial", int(BALL_RADIUS * 0.8)),
                angle=math.degrees(ball.rotation)
            )

def main():
    root = tk.Tk()
    root.title("Balls Bouncing in a Spinning Heptagon")
    
    simulation = PhysicsSimulation(root)
    
    root.mainloop()

if __name__ == "__main__":
    main()