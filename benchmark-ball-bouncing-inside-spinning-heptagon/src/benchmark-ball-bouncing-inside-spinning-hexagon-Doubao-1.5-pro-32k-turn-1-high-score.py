import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
HEPTAGON_RADIUS = 300
BALL_RADIUS = 20
GRAVITY = 0.1
FRICTION = 0.99
SPIN_SPEED = 2 * math.pi / 5  # 360 degrees per 5 seconds in radians per frame
COLORS = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d",
    "#ec6800", "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101",
    "#e49e61", "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]
NUM_BALLS = 20


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    spin: float
    color: str
    number: int


def create_heptagon():
    vertices = []
    for i in range(7):
        angle = 2 * math.pi * i / 7
        x = CENTER_X + HEPTAGON_RADIUS * math.cos(angle)
        y = CENTER_Y + HEPTAGON_RADIUS * math.sin(angle)
        vertices.extend([x, y])
    return canvas.create_polygon(vertices, fill="", outline="black")


def create_balls():
    balls = []
    for i in range(NUM_BALLS):
        ball = Ball(
            x=CENTER_X,
            y=CENTER_Y,
            vx=0,
            vy=0,
            spin=0,
            color=COLORS[i],
            number=i + 1
        )
        balls.append(ball)
    return balls


def draw_balls(balls):
    for ball in balls:
        x1 = ball.x - BALL_RADIUS
        y1 = ball.y - BALL_RADIUS
        x2 = ball.x + BALL_RADIUS
        y2 = ball.y + BALL_RADIUS
        canvas.create_oval(x1, y1, x2, y2, fill=ball.color)
        canvas.create_text(ball.x, ball.y, text=str(ball.number))


def rotate_heptagon(angle):
    vertices = []
    for i in range(7):
        vertex_angle = 2 * math.pi * i / 7 + angle
        x = CENTER_X + HEPTAGON_RADIUS * math.cos(vertex_angle)
        y = CENTER_Y + HEPTAGON_RADIUS * math.sin(vertex_angle)
        vertices.extend([x, y])
    canvas.coords(heptagon, vertices)


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def ball_ball_collision(balls):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            ball1 = balls[i]
            ball2 = balls[j]
            dist = distance(ball1.x, ball1.y, ball2.x, ball2.y)
            if dist <= 2 * BALL_RADIUS:
                # Collision response
                normal = np.array([ball2.x - ball1.x, ball2.y - ball1.y])
                normal = normal / np.linalg.norm(normal)
                relative_velocity = np.array([ball2.vx - ball1.vx, ball2.vy - ball1.vy])
                impulse = np.dot(relative_velocity, normal)
                if impulse > 0:
                    ball1.vx += impulse * normal[0]
                    ball1.vy += impulse * normal[1]
                    ball2.vx -= impulse * normal[0]
                    ball2.vy -= impulse * normal[1]
                    # Add spin effect
                    ball1.spin += impulse * 0.1
                    ball2.spin -= impulse * 0.1


def ball_wall_collision(balls, angle):
    for ball in balls:
        for i in range(7):
            start_angle = 2 * math.pi * i / 7 + angle
            end_angle = 2 * math.pi * (i + 1) / 7 + angle
            start_x = CENTER_X + HEPTAGON_RADIUS * math.cos(start_angle)
            start_y = CENTER_Y + HEPTAGON_RADIUS * math.sin(start_angle)
            end_x = CENTER_X + HEPTAGON_RADIUS * math.cos(end_angle)
            end_y = CENTER_Y + HEPTAGON_RADIUS * math.sin(end_angle)
            # Calculate the distance from the ball to the line segment
            numerator = abs((end_y - start_y) * ball.x - (end_x - start_x) * ball.y + end_x * start_y - end_y * start_x)
            denominator = distance(start_x, start_y, end_x, end_y)
            dist = numerator / denominator
            if dist <= BALL_RADIUS:
                # Collision response
                normal = np.array([math.cos(start_angle + math.pi / 2), math.sin(start_angle + math.pi / 2)])
                relative_velocity = np.array([ball.vx, ball.vy])
                impulse = np.dot(relative_velocity, normal)
                if impulse > 0:
                    ball.vx -= 2 * impulse * normal[0]
                    ball.vy -= 2 * impulse * normal[1]
                    # Add spin effect
                    ball.spin += impulse * 0.1


def update_balls(balls):
    for ball in balls:
        ball.vy += GRAVITY
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        ball.spin *= FRICTION
        ball.x += ball.vx
        ball.y += ball.vy


def main_loop(angle):
    canvas.delete("all")
    rotate_heptagon(angle)
    ball_ball_collision(balls)
    ball_wall_collision(balls, angle)
    update_balls(balls)
    draw_balls(balls)
    new_angle = angle + SPIN_SPEED
    root.after(10, main_loop, new_angle)


root = tk.Tk()
root.title("Bouncing Balls in Spinning Heptagon")
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
canvas.pack()

heptagon = create_heptagon()
balls = create_balls()

main_loop(0)

root.mainloop()