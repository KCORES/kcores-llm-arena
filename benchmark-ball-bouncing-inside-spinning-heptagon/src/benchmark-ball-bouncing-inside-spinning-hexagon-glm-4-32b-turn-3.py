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
FRICTION_COEFFICIENT = 0.3
RESTITUTION_COEFFICIENT = 0.7  # Controls bounce height
ANGULAR_VELOCITY = 2 * math.pi / 5  # 360 degrees per 5 seconds
BALL_RADIUS = 15
HEPTAGON_RADIUS = 300  # Distance from center to vertices
BALL_COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d", "#ec6800", 
    "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61", "#e45e32", 
    "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]
FPS = 60
TIME_STEP = 1.0 / FPS

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
        
        # Update rotation based on angular velocity
        self.rotation += self.angular_velocity * dt
        
        # Apply gravity
        self.velocity.y += GRAVITY * dt

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
        self.rotation += ANGULAR_VELOCITY * dt
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
            # All balls start at the center with small random velocities
            position = Vector2D(center.x, center.y)
            velocity = Vector2D(
                (np.random.random() - 0.5) * 50,  # Small random x velocity
                (np.random.random() - 0.5) * 50   # Small random y velocity
            )
            angular_velocity = (np.random.random() - 0.5) * 2  # Random initial spin
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
        self.root.after(int(TIME_STEP * 1000), self.update)
    
    def check_ball_wall_collisions(self):
        for ball in self.balls:
            for edge in self.heptagon.edges:
                # Calculate the closest point on the line segment to the ball center
                edge_vector = Vector2D(edge[1].x - edge[0].x, edge[1].y - edge[0].y)
                ball_to_edge_start = Vector2D(ball.position.x - edge[0].x, ball.position.y - edge[0].y)
                
                # Project ball_to_edge_start onto edge_vector
                edge_length_squared = edge_vector.x**2 + edge_vector.y**2
                t = max(0, min(1, ball_to_edge_start.dot(edge_vector) / edge_length_squared))
                
                closest_point = Vector2D(
                    edge[0].x + t * edge_vector.x,
                    edge[0].y + t * edge_vector.y
                )
                
                # Calculate distance from ball center to closest point
                distance_vector = Vector2D(
                    ball.position.x - closest_point.x,
                    ball.position.y - closest_point.y
                )
                distance = distance_vector.length()
                
                # Check for collision
                if distance <= BALL_RADIUS:
                    # Calculate normal vector (pointing from wall to ball)
                    normal = distance_vector.normalize()
                    
                    # Calculate relative velocity
                    relative_velocity = ball.velocity
                    
                    # Calculate velocity component along the normal
                    normal_velocity = normal.dot(relative_velocity)
                    
                    # Only resolve collision if objects are moving toward each other
                    if normal_velocity < 0:
                        # Calculate impulse
                        impulse = -(1 + RESTITUTION_COEFFICIENT) * normal_velocity
                        
                        # Apply impulse to ball's velocity
                        ball.velocity = ball.velocity + normal * impulse
                        
                        # Calculate tangent vector
                        tangent = Vector2D(-normal.y, normal.x)
                        
                        # Calculate velocity component along the tangent
                        tangent_velocity = tangent.dot(relative_velocity)
                        
                        # Apply friction impulse
                        friction_impulse = -FRICTION_COEFFICIENT * tangent_velocity
                        ball.velocity = ball.velocity + tangent * friction_impulse
                        
                        # Apply angular impulse (from friction)
                        ball.angular_velocity += friction_impulse * BALL_RADIUS / ball.moment_of_inertia
                        
                        # Move ball outside of wall to prevent sticking
                        penetration_depth = BALL_RADIUS - distance
                        ball.position = ball.position + normal * penetration_depth
    
    def check_ball_ball_collisions(self):
        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                ball1 = self.balls[i]
                ball2 = self.balls[j]
                
                # Calculate distance between ball centers
                distance_vector = Vector2D(
                    ball2.position.x - ball1.position.x,
                    ball2.position.y - ball1.position.y
                )
                distance = distance_vector.length()
                
                # Check for collision
                if distance <= 2 * BALL_RADIUS:
                    # Calculate normal vector
                    normal = distance_vector.normalize()
                    
                    # Calculate relative velocity
                    relative_velocity = Vector2D(
                        ball2.velocity.x - ball1.velocity.x,
                        ball2.velocity.y - ball1.velocity.y
                    )
                    
                    # Calculate velocity component along the normal
                    normal_velocity = normal.dot(relative_velocity)
                    
                    # Only resolve collision if objects are moving toward each other
                    if normal_velocity < 0:
                        # Calculate impulse
                        impulse = -(1 + RESTITUTION_COEFFICIENT) * normal_velocity / 2
                        
                        # Apply impulse to both balls
                        ball1.velocity = ball1.velocity - normal * impulse
                        ball2.velocity = ball2.velocity + normal * impulse
                        
                        # Calculate tangent vector
                        tangent = Vector2D(-normal.y, normal.x)
                        
                        # Calculate velocity component along the tangent
                        tangent_velocity = tangent.dot(relative_velocity)
                        
                        # Apply friction impulse
                        friction_impulse = -FRICTION_COEFFICIENT * tangent_velocity / 2
                        ball1.velocity = ball1.velocity - tangent * friction_impulse
                        ball2.velocity = ball2.velocity + tangent * friction_impulse
                        
                        # Apply angular impulse (from friction)
                        ball1.angular_velocity -= friction_impulse * BALL_RADIUS / ball1.moment_of_inertia
                        ball2.angular_velocity += friction_impulse * BALL_RADIUS / ball2.moment_of_inertia
                        
                        # Move balls apart to prevent sticking
                        penetration_depth = 2 * BALL_RADIUS - distance
                        ball1.position = ball1.position - normal * (penetration_depth / 2)
                        ball2.position = ball2.position + normal * (penetration_depth / 2)
    
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
                fill="black",
                font=("Arial", int(BALL_RADIUS * 0.8)),
                angle=math.degrees(ball.rotation)
            )

def main():
    root = tk.Tk()
    root.title("Bouncing Balls in a Spinning Heptagon")
    
    simulation = PhysicsSimulation(root)
    
    root.mainloop()

if __name__ == "__main__":
    main()