import tkinter as tk
import math
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple
import time

@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    rotation: float
    angular_velocity: float
    radius: float
    color: str
    number: int

class Heptagon:
    def __init__(self, center_x, center_y, radius, rotation_speed):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.rotation = 0.0
        self.rotation_speed = rotation_speed  # radians per second
        self.vertices = self.calculate_vertices()

    def calculate_vertices(self):
        vertices = []
        angle = self.rotation
        for _ in range(7):
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            vertices.append((x, y))
            angle += 2 * math.pi / 7
        return vertices

    def rotate(self, dt):
        self.rotation += self.rotation_speed * dt
        self.vertices = self.calculate_vertices()

    def get_edges(self):
        edges = []
        n = len(self.vertices)
        for i in range(n):
            edges.append((self.vertices[i], self.vertices[(i+1)%n]))
        return edges

def update(dt):
    heptagon.rotate(dt)
    
    for ball in balls:
        ball.vy += gravity * dt
        ball.vx *= friction
        ball.vy *= friction
        ball.x += ball.vx * dt
        ball.y += ball.vy * dt
        ball.angular_velocity *= angular_friction
        ball.rotation += ball.angular_velocity * dt

    edges = heptagon.get_edges()
    for ball in balls:
        for edge in edges:
            p1, p2 = edge
            edge_vec = np.array([p2[0]-p1[0], p2[1]-p1[1]])
            edge_length = np.linalg.norm(edge_vec)
            if edge_length == 0:
                continue
            edge_unit = edge_vec / edge_length
            to_ball = np.array([ball.x - p1[0], ball.y - p1[1]])
            projection = np.dot(to_ball, edge_unit)
            projection = max(0, min(edge_length, projection))
            closest_point = p1[0] + edge_unit[0] * projection, p1[1] + edge_unit[1] * projection
            dx = ball.x - closest_point[0]
            dy = ball.y - closest_point[1]
            dist_sq = dx*dx + dy*dy
            if dist_sq < ball.radius**2:
                normal = np.array([-edge_unit[1], edge_unit[0]])
                penetration = ball.radius - math.sqrt(dist_sq)
                overlap = normal * penetration
                ball.x += overlap[0]
                ball.y += overlap[1]
                vel = np.array([ball.vx, ball.vy])
                reflection = vel - 2 * np.dot(vel, normal) * normal
                ball.vx = reflection[0] * elasticity
                ball.vy = reflection[1] * elasticity
                ball.angular_velocity += np.random.uniform(-0.2, 0.2)

    for i in range(len(balls)):
        for j in range(i+1, len(balls)):
            b1, b2 = balls[i], balls[j]
            dx = b1.x - b2.x
            dy = b1.y - b2.y
            dist_sq = dx*dx + dy*dy
            if dist_sq < (2*b1.radius)**2:
                dist = math.sqrt(dist_sq)
                overlap = 2*b1.radius - dist
                unit_x = dx / dist
                unit_y = dy / dist
                b1.x += unit_x * overlap * 0.5
                b1.y += unit_y * overlap * 0.5
                b2.x -= unit_x * overlap * 0.5
                b2.y -= unit_y * overlap * 0.5
                vel1 = np.array([b1.vx, b1.vy])
                vel2 = np.array([b2.vx, b2.vy])
                rv = vel1 - vel2
                normal = np.array([dx, dy]) / dist
                dot = np.dot(rv, normal)
                j = (1 + elasticity) * dot / 2
                impulse = j * normal
                b1.vx -= impulse[0]
                b1.vy -= impulse[1]
                b2.vx += impulse[0]
                b2.vy += impulse[1]

def draw():
    canvas.delete("all")
    canvas.create_polygon(heptagon.vertices, outline='black', fill='', width=2)
    for ball in balls:
        x1 = ball.x - ball.radius
        y1 = ball.y - ball.radius
        x2 = ball.x + ball.radius
        y2 = ball.y + ball.radius
        canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline='')
        canvas.create_text(ball.x, ball.y, text=str(ball.number), fill='black', font=('Arial', 12))

def animate():
    global last_time
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    update(dt)
    draw()
    root.after(1, animate)

root = tk.Tk()
canvas = tk.Canvas(root, width=800, height=800, bg='white')
canvas.pack()

heptagon = Heptagon(400, 400, 350, 2 * math.pi / 5)
colors = [
    "#f8b862", "#f6ad49", "#f39800", "#f08300", "#ec6d51", "#ee7948", "#ed6d3d",
    "#ec6800", "#ec6800", "#ee7800", "#eb6238", "#ea5506", "#ea5506", "#eb6101",
    "#e49e61", "#e45e32", "#e17b34", "#dd7a56", "#db8449", "#d66a35"
]

balls = []
for i in range(20):
    balls.append(Ball(
        x=400, y=400,
        vx=0, vy=0,
        rotation=0, angular_velocity=0,
        radius=15,
        color=colors[i],
        number=i+1
    ))

gravity = 9.8
friction = 0.99
angular_friction = 0.99
elasticity = 0.8
last_time = time.time()

animate()
root.mainloop()