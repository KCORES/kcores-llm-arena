import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 360 / (5 * 60)  # Degrees per frame (60 FPS)
COLORS = [
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ec6800",
    "#ee7800",
    "#eb6238",
    "#ea5506",
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
    "#db8449",
    "#d66a35",
]


# Data classes
@dataclass
class Ball:
    id: int
    x: float
    y: float
    vx: float
    vy: float
    angle: float
    spin: float


@dataclass
class Heptagon:
    rotation: float


# Initialize balls
def create_balls() -> List[Ball]:
    balls = []
    for i in range(20):
        angle = (i / 20) * 2 * math.pi
        x = WIDTH / 2 + math.cos(angle) * BALL_RADIUS
        y = HEIGHT / 2 + math.sin(angle) * BALL_RADIUS
        balls.append(Ball(i + 1, x, y, 0, 0, angle, 0))
    return balls


# Heptagon vertices
def get_heptagon_vertices(rotation: float) -> List[tuple]:
    vertices = []
    for i in range(7):
        angle = math.radians(rotation + i * 360 / 7)
        x = WIDTH / 2 + math.cos(angle) * HEPTAGON_RADIUS
        y = HEIGHT / 2 + math.sin(angle) * HEPTAGON_RADIUS
        vertices.append((x, y))
    return vertices


# Collision detection between balls
def check_ball_collisions(balls: List[Ball]):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            dx = balls[i].x - balls[j].x
            dy = balls[i].y - balls[j].y
            distance = math.hypot(dx, dy)
            if distance < 2 * BALL_RADIUS:
                # Elastic collision response
                angle = math.atan2(dy, dx)
                speed1 = math.hypot(balls[i].vx, balls[i].vy)
                speed2 = math.hypot(balls[j].vx, balls[j].vy)
                direction1 = math.atan2(balls[i].vy, balls[i].vx)
                direction2 = math.atan2(balls[j].vy, balls[j].vx)
                new_vx1 = speed1 * math.cos(direction1 - angle)
                new_vy1 = speed1 * math.sin(direction1 - angle)
                new_vx2 = speed2 * math.cos(direction2 - angle)
                new_vy2 = speed2 * math.sin(direction2 - angle)
                final_vx1 = (
                    speed1 * math.cos(direction1 - angle) * (1 - 1)
                    + 2 * speed2 * math.cos(direction2 - angle)
                ) / 2
                final_vx2 = (
                    speed2 * math.cos(direction2 - angle) * (1 - 1)
                    + 2 * speed1 * math.cos(direction1 - angle)
                ) / 2
                balls[i].vx = (
                    math.cos(angle) * final_vx1
                    + math.cos(angle + math.pi / 2) * new_vy1
                )
                balls[i].vy = (
                    math.sin(angle) * final_vx1
                    + math.sin(angle + math.pi / 2) * new_vy1
                )
                balls[j].vx = (
                    math.cos(angle) * final_vx2
                    + math.cos(angle + math.pi / 2) * new_vy2
                )
                balls[j].vy = (
                    math.sin(angle) * final_vx2
                    + math.sin(angle + math.pi / 2) * new_vy2
                )


# Update ball positions
def update_balls(balls: List[Ball], heptagon: Heptagon):
    for ball in balls:
        # Apply gravity
        ball.vy += GRAVITY
        # Apply friction
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        # Update position
        ball.x += ball.vx
        ball.y += ball.vy
        # Update spin
        ball.angle += ball.spin
        ball.spin *= FRICTION
        # Check collision with heptagon walls
        vertices = get_heptagon_vertices(heptagon.rotation)
        for i in range(7):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % 7]
            # Line segment from (x1, y1) to (x2, y2)
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            nx = -dy / length
            ny = dx / length
            # Distance from ball to line
            dist = (ball.x - x1) * nx + (ball.y - y1) * ny
            if abs(dist) < BALL_RADIUS:
                # Reflect velocity
                dot = ball.vx * nx + ball.vy * ny
                ball.vx -= 2 * dot * nx
                ball.vy -= 2 * dot * ny
                # Move ball outside the wall
                ball.x += (BALL_RADIUS - dist) * nx
                ball.y += (BALL_RADIUS - dist) * ny


# Main animation loop
def animate(canvas: tk.Canvas, balls: List[Ball], heptagon: Heptagon):
    canvas.delete("all")
    # Draw heptagon
    vertices = get_heptagon_vertices(heptagon.rotation)
    canvas.create_polygon(vertices, outline="black", fill="")
    # Draw balls
    for ball in balls:
        x1 = ball.x - BALL_RADIUS
        y1 = ball.y - BALL_RADIUS
        x2 = ball.x + BALL_RADIUS
        y2 = ball.y + BALL_RADIUS
        canvas.create_oval(x1, y1, x2, y2, fill=COLORS[ball.id - 1])
        canvas.create_text(ball.x, ball.y, text=str(ball.id), font=("Arial", 10))
    # Update physics
    check_ball_collisions(balls)
    update_balls(balls, heptagon)
    heptagon.rotation += SPIN_SPEED
    # Repeat
    canvas.after(16, animate, canvas, balls, heptagon)


# Main function
def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()
    balls = create_balls()
    heptagon = Heptagon(0)
    animate(canvas, balls, heptagon)
    root.mainloop()


if __name__ == "__main__":
    main()
