import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
import sys

# Constants
WIDTH = 800
HEIGHT = 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 10
GRAVITY = 0.5  # pixels per frame squared
FRICTION = 0.99  # air friction
RESTITUTION = 0.5  # coefficient of restitution for walls
BALL_FRICTION = 0.3  # friction between balls
ROTATION_SPEED_DEG = 360 / 5  # degrees per second
COLORS = [
    '#f8b862', '#f6ad49', '#f39800', '#f08300', '#ec6d51',
    '#ee7948', '#ed6d3d', '#ec6800', '#ec6800', '#ee7800',
    '#eb6238', '#ea5506', '#ea5506', '#eb6101', '#e49e61',
    '#e45e32', '#e17b34', '#dd7a56', '#db8449', '#d66a35'
]

@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    angular_velocity: float = 0.0  # radians per frame

def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def closest_point_on_segment(p, a, b):
    ap = np.array(p) - np.array(a)
    ab = np.array(b) - np.array(a)
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(t, 0, 1)
    return np.array(a) + t * ab

def check_heptagon_collision(ball, heptagon_center, heptagon_radius, edges, angular_velocity):
    cx, cy = heptagon_center
    r = heptagon_radius
    handled = False
    for i in range(7):
        p1 = edges[i]
        p2 = edges[(i+1)%7]
        closest = closest_point_on_segment((ball.x, ball.y), p1, p2)
        dx = ball.x - closest[0]
        dy = ball.y - closest[1]
        dist = math.hypot(dx, dy)
        if dist < ball.radius - 1:  # Slight epsilon to prevent sticking
            # Collision detected
            edge_dx = p2[0] - p1[0]
            edge_dy = p2[1] - p1[1]
            edge_vec = np.array([edge_dx, edge_dy])
            edge_length = np.linalg.norm(edge_vec)
            if edge_length == 0:
                continue
            edge_vec /= edge_length
            
            # Calculate inward normal
            center_vec = np.array([cx - p1[0], cy - p1[1]])
            perp1 = np.array([-edge_dy, edge_dx])
            perp2 = np.array([edge_dy, -edge_dx])
            dot1 = np.dot(perp1, center_vec)
            normal = perp1 if dot1 > 0 else perp2
            normal /= np.linalg.norm(normal)
            
            # Calculate wall velocity at collision point
            tangent = np.array([-normal[1], normal[0]])
            v_wall = angular_velocity * r * tangent
            
            # Relative velocity
            v_ball = np.array([ball.vx, ball.vy])
            v_rel = v_ball - v_wall
            
            # Collision response
            vn_rel = np.dot(v_rel, normal)
            if vn_rel < 0:  # Only collide if moving towards the wall
                impulse = -(1 + RESTITUTION) * vn_rel
                ball.vx += impulse * normal[0]
                ball.vy += impulse * normal[1]
                
                # Position correction
                penetration = ball.radius - dist
                ball.x = closest[0] + normal[0] * (ball.radius + penetration)
                ball.y = closest[1] + normal[1] * (ball.radius + penetration)
                handled = True
    return handled

def check_ball_collisions(balls):
    for i in range(len(balls)):
        for j in range(i+1, len(balls)):
            b1 = balls[i]
            b2 = balls[j]
            dx = b2.x - b1.x
            dy = b2.y - b1.y
            dist = math.hypot(dx, dy)
            if dist < 2 * BALL_RADIUS:
                # Collision response
                nx = dx / dist
                ny = dy / dist
                tx = -ny
                ty = nx
                
                # Calculate relative velocity
                v1n = b1.vx * nx + b1.vy * ny
                v2n = b2.vx * nx + b2.vy * ny
                v_rel = v2n - v1n
                
                # Impulse calculation
                impulse = (1 + RESTITUTION) * v_rel / 2
                b1.vx -= impulse * nx
                b1.vy -= impulse * ny
                b2.vx += impulse * nx
                b2.vy += impulse * ny
                
                # Position correction
                overlap = 2 * BALL_RADIUS - dist
                b1.x -= nx * overlap * 0.5
                b1.y -= ny * overlap * 0.5
                b2.x += nx * overlap * 0.5
                b2.y += ny * overlap * 0.5
                
                # Friction
                v1t = b1.vx * tx + b1.vy * ty
                v2t = b2.vx * tx + b2.vy * ty
                impulse_friction = BALL_FRICTION * (v2t - v1t) / 2
                b1.vx -= impulse_friction * tx
                b1.vy -= impulse_friction * ty
                b2.vx += impulse_friction * tx
                b2.vy += impulse_friction * ty

def update_frame(canvas, heptagon_center, heptagon_radius, balls, angle, angular_velocity):
    canvas.delete("all")
    
    # Update heptagon angle
    angle += angular_velocity
    
    # Draw heptagon
    edges = []
    for i in range(7):
        theta = angle + i * 2 * math.pi / 7
        x = heptagon_center[0] + heptagon_radius * math.cos(theta)
        y = heptagon_center[1] + heptagon_radius * math.sin(theta)
        edges.append((x, y))
        if i == 0:
            canvas.create_line(x, y, edges[-1][0], edges[-1][1], fill='black', width=2)
        else:
            canvas.create_line(edges[i-1][0], edges[i-1][1], x, y, fill='black', width=2)
    
    # Update balls
    for ball in balls:
        # Apply gravity
        ball.vy += GRAVITY
        # Apply air friction
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        # Update position
        ball.x += ball.vx
        ball.y += ball.vy
        # Check collisions with heptagon
        check_heptagon_collision(ball, heptagon_center, heptagon_radius, edges, angular_velocity)
    
    # Check collisions between balls
    check_ball_collisions(balls)
    
    # Draw balls
    for ball in balls:
        x0 = ball.x - BALL_RADIUS
        y0 = ball.y - BALL_RADIUS
        canvas.create_oval(x0, y0, x0 + 2*BALL_RADIUS, y0 + 2*BALL_RADIUS, 
                          fill=ball.color, outline='')
        # Draw number with spin effect
        canvas.create_text(ball.x, ball.y, text=str(ball.number), 
                          font=('Arial', 10), fill='white')
    
    canvas.after(16, update_frame, canvas, heptagon_center, heptagon_radius, 
                balls, angle, angular_velocity)

def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg='white')
    canvas.pack()
    
    # Initialize balls
    heptagon_center = (WIDTH/2, HEIGHT/2)
    balls = []
    for i in range(20):
        balls.append(Ball(
            x=heptagon_center[0],
            y=heptagon_center[1],
            vx=np.random.uniform(-1, 1),  # Initial random spin
            vy=0,
            radius=BALL_RADIUS,
            color=COLORS[i],
            number=i+1
        ))
    
    # Initial angle and angular velocity (converted to radians per frame)
    angle = 0
    angular_velocity = math.radians(ROTATION_SPEED_DEG) / 60  # 60 FPS assumption
    
    update_frame(canvas, heptagon_center, HEPTAGON_RADIUS, balls, angle, angular_velocity)
    root.mainloop()

if __name__ == "__main__":
    main()