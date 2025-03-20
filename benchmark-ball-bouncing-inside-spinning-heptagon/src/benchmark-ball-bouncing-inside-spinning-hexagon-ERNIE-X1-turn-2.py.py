import tkinter as tk
import math
import time
from dataclasses import dataclass

# Configuration constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51",
    "#ee7948", "#ed6d3d", "#ec6800", "#ec6800", "#ee7800",
    "#eb6238", "#ea5506", "#ea5506", "#eb6101", "#e49e61",
    "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]
GRAVITY = 400.0       # pixels/s²
AIR_FRICTION = 0.995  # per frame
BOUNCE_FACTOR = 0.85  # wall bounce elasticity
BALL_FRICTION = 0.8   # ball collision friction
SPIN_DECAY = 0.95     # spin decay factor

@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 12
    mass: float = 1.0
    color: str = "white"
    number: int = 0
    omega: float = 0.0  # angular velocity in radians/frame

def create_heptagon(radius, center_x, center_y, rotation=0):
    angles = [i * 2 * math.pi / 7 + rotation for i in range(7)]
    return [(center_x + radius * math.cos(a), center_y + radius * math.sin(a)) for a in angles]

def distance_point_to_segment(px, py, ax, ay, bx, by):
    # Calculate closest point on segment to point (px, py)
    dx = bx - ax
    dy = by - ay
    if dx == dy == 0:
        return math.hypot(px - ax, py - ay)
    
    t = ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy)
    t = max(0, min(1, t))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)

def resolve_wall_collision(ball, p1, p2, dt):
    # Find closest point on wall segment
    ax, ay = p1
    bx, by = p2
    closest_dist = distance_point_to_segment(ball.x, ball.y, ax, ay, bx, by)
    
    if closest_dist > ball.radius:
        return False
    
    # Calculate normal vector
    dx = bx - ax
    dy = by - ay
    length = math.hypot(dx, dy)
    if length == 0:
        return False
    nx = -dy / length
    ny = dx / length
    
    # Calculate relative velocity
    wall_speed = 2 * math.pi / 5  # radians/second
    wall_vx = -wall_speed * by
    wall_vy = wall_speed * bx
    rel_vx = ball.vx - wall_vx
    rel_vy = ball.vy - wall_vy
    
    # Calculate velocity components
    dot = rel_vx * nx + rel_vy * ny
    if dot > 0:
        return False  # Moving away from wall
    
    # Apply bounce
    ball.vx = (-BOUNCE_FACTOR * dot * nx) + wall_vx
    ball.vy = (-BOUNCE_FACTOR * dot * ny) + wall_vy
    
    # Apply tangential friction
    tangent_x = -ny
    tangent_y = nx
    tangent_vel = rel_vx * tangent_x + rel_vy * tangent_y
    ball.vx += tangent_vel * (1 - BALL_FRICTION) * tangent_x
    ball.vy += tangent_vel * (1 - BALL_FRICTION) * tangent_y
    
    # Add spin from collision
    ball.omega += math.copysign(1, tangent_vel) * 0.5
    return True

def resolve_ball_collision(ball1, ball2):
    dx = ball2.x - ball1.x
    dy = ball2.y - ball1.y
    dist_sq = dx*dx + dy*dy
    if dist_sq < (ball1.radius + ball2.radius)**2:
        dist = math.sqrt(dist_sq)
        nx = dx / dist
        ny = dy / dist
        
        # Calculate relative velocity
        v1_rel_x = ball1.vx - ball2.vx
        v1_rel_y = ball1.vy - ball2.vy
        dot = v1_rel_x * nx + v1_rel_y * ny
        
        if dot >= 0:
            return  # Already moving apart
        
        # Calculate impulse
        impulse = (2 * dot) / (ball1.mass + ball2.mass)
        ball1.vx -= impulse * ball2.mass * nx
        ball1.vy -= impulse * ball2.mass * ny
        ball2.vx += impulse * ball1.mass * nx
        ball2.vy += impulse * ball1.mass * ny
        
        # Apply friction
        tangent_x = -ny
        tangent_y = nx
        tangent_vel = v1_rel_x * tangent_x + v1_rel_y * tangent_y
        friction_impulse = BALL_FRICTION * tangent_vel
        
        ball1.vx -= friction_impulse * tangent_x
        ball1.vy -= friction_impulse * tangent_y
        ball2.vx += friction_impulse * tangent_x
        ball2.vy += friction_impulse * tangent_y
        
        # Transfer spin
        ball1.omega += math.copysign(0.5, tangent_vel)
        ball2.omega += math.copysign(0.5, -tangent_vel)

def update_balls(balls, vertices, dt):
    # Apply gravity and air friction
    for ball in balls:
        ball.vy += GRAVITY * dt
        ball.vx *= AIR_FRICTION
        ball.vy *= AIR_FRICTION
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        ball.omega *= SPIN_DECAY
    
    # Handle wall collisions
    for ball in balls:
        for i in range(7):
            p1 = vertices[i]
            p2 = vertices[(i+1)%7]
            resolve_wall_collision(ball, p1, p2, dt)
    
    # Handle ball collisions
    for i in range(len(balls)):
        for j in range(i+1, len(balls)):
            resolve_ball_collision(balls[i], balls[j])

def draw_heptagon(canvas, vertices):
    canvas.create_polygon(
        [x for x, y in vertices] + [vertices[0][0]],
        [y for x, y in vertices] + [vertices[0][1]],
        outline="black", fill="#333333", width=2
    )

def draw_balls(canvas, balls):
    for ball in balls:
        # Draw ball body
        canvas.create_oval(
            ball.x - ball.radius,
            ball.y - ball.radius,
            ball.x + ball.radius,
            ball.y + ball.radius,
            fill=ball.color, outline=""
        )
        # Draw number with spin
        canvas.save()
        canvas.translate(ball.x, ball.y)
        canvas.rotate(math.degrees(ball.omega * 180 / math.pi))
        canvas.create_text(0, 0, text=str(ball.number), 
                          font=("Arial", 10, "bold"), fill="white")
        canvas.restore()

def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="#ffffff")
    canvas.pack()
    
    # Initialize balls
    balls = []
    for i in range(20):
        angle = i * math.pi * 2 / 20
        balls.append(Ball(
            CENTER_X, CENTER_Y,
            math.cos(angle) * 5,
            math.sin(angle) * 5,
            color=COLORS[i],
            number=i+1
        ))
    
    last_time = time.time()
    rotation_angle = 0
    
    def animate():
        nonlocal last_time, rotation_angle
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Update heptagon rotation
        rotation_angle += (2 * math.pi / 5) * dt
        rotation_angle %= (2 * math.pi)
        vertices = create_heptagon(HEPTAGON_RADIUS, CENTER_X, CENTER_Y, rotation_angle)
        
        # Update physics
        update_balls(balls, vertices, dt)
        
        # Draw frame
        canvas.delete("all")
        draw_heptagon(canvas, vertices)
        draw_balls(canvas, balls)
        
        root.after(16, animate)  # ~60 FPS
    
    animate()
    root.mainloop()

if __name__ == "__main__":
    main()