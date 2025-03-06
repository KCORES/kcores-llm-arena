import tkinter as tk
import math
import random
from dataclasses import dataclass
from typing import List

WIDTH, HEIGHT = 800, 800
BALL_RADIUS = 10
HEX_RADIUS = 300
GRAVITY_ACCEL = 0.5  # pixels per frame squared
FRICTION = 0.99  # velocity damping factor per frame
SPIN_FRICTION = 0.98  # angular velocity damping factor per frame

BALL_COLORS = [
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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: int
    color: str
    number: int
    spin_angle: float = 0.0
    angular_velocity: float = 0.0  # degrees per frame


def distance_to_segment(P, A, B):
    ax, ay = A
    bx, by = B
    px, py = P
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    dot = apx * abx + apy * aby
    if dot <= 0:
        return math.hypot(px - ax, py - ay)
    len_sq = abx**2 + aby**2
    if dot >= len_sq:
        return math.hypot(px - bx, py - by)
    t = dot / len_sq
    proj_x = ax + t * abx
    proj_y = ay + t * aby
    return math.hypot(px - proj_x, py - proj_y)


class PhysicsEngine:
    def __init__(self):
        self.balls = []
        self.angle = 0.0
        self.angular_velocity = 2 * math.pi / 5  # radians per second (360°/5s)

    def create_balls(self):
        center_x = WIDTH / 2
        center_y = HEIGHT / 2
        for i in range(20):
            self.balls.append(
                Ball(
                    x=center_x,
                    y=center_y,
                    vx=random.uniform(-0.1, 0.1),
                    vy=random.uniform(-0.1, 0.1),
                    radius=BALL_RADIUS,
                    color=BALL_COLORS[i],
                    number=i + 1,
                )
            )

    def get_heptagon_vertices(self):
        center_x, center_y = WIDTH / 2, HEIGHT / 2
        vertices = []
        for i in range(7):
            theta = i * 2 * math.pi / 7 + self.angle
            x = center_x + HEX_RADIUS * math.cos(theta)
            y = center_y + HEX_RADIUS * math.sin(theta)
            vertices.append((x, y))
        return vertices

    def update(self, delta_time):
        self.angle += self.angular_velocity * delta_time
        self.angle %= 2 * math.pi

        for ball in self.balls:
            ball.vy += GRAVITY_ACCEL
            ball.vx *= FRICTION
            ball.vy *= FRICTION
            ball.x += ball.vx
            ball.y += ball.vy

            ball.spin_angle += ball.angular_velocity * delta_time
            ball.angular_velocity *= SPIN_FRICTION

        # Ball-ball collisions
        for i in range(len(self.balls)):
            a = self.balls[i]
            for j in range(i + 1, len(self.balls)):
                b = self.balls[j]
                dx = a.x - b.x
                dy = a.y - b.y
                dist_sq = dx * dx + dy * dy
                if dist_sq < (2 * BALL_RADIUS) ** 2 and dist_sq > 0:
                    dist = math.sqrt(dist_sq)
                    nx = dx / dist
                    ny = dy / dist
                    rvx = a.vx - b.vx
                    rvy = a.vy - b.vy
                    rv_dot_n = rvx * nx + rvy * ny
                    j_val = -(1 + 1.0) * rv_dot_n / 2.0
                    jx = j_val * nx
                    jy = j_val * ny
                    a.vx -= jx
                    a.vy -= jy
                    b.vx += jx
                    b.vy += jy

                    move = (2 * BALL_RADIUS - dist) * 0.5
                    a.x += nx * move * 0.5
                    a.y += ny * move * 0.5
                    b.x -= nx * move * 0.5
                    b.y -= ny * move * 0.5

                    a.angular_velocity += a.number * 0.1
                    b.angular_velocity += b.number * 0.1

        # Wall collisions
        vertices = self.get_heptagon_vertices()
        edges = [(vertices[i], vertices[(i + 1) % 7]) for i in range(7)]

        for ball in self.balls:
            closest_dist = float("inf")
            best_edge = None
            for edge in edges:
                a, b = edge
                dist = distance_to_segment((ball.x, ball.y), a, b)
                if dist < ball.radius and dist < closest_dist:
                    closest_dist = dist
                    best_edge = edge
            if best_edge:
                a, b = best_edge
                ax, ay = a
                bx, by = b
                dx_edge = bx - ax
                dy_edge = by - ay
                length = math.hypot(dx_edge, dy_edge)
                normal_x = (-dy_edge) / length
                normal_y = dx_edge / length

                vel_dot_n = ball.vx * normal_x + ball.vy * normal_y
                ball.vx -= 2 * vel_dot_n * normal_x
                ball.vy -= 2 * vel_dot_n * normal_y

                penetration = ball.radius - closest_dist
                ball.x += normal_x * penetration
                ball.y += normal_y * penetration

                ball.angular_velocity += ball.number * 0.3


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
    canvas.pack()

    engine = PhysicsEngine()
    engine.create_balls()

    delta_time = 1 / 60

    def animate():
        engine.update(delta_time)
        canvas.delete("all")

        # Draw spinning heptagon
        vertices = engine.get_heptagon_vertices()
        canvas.create_polygon(vertices, outline="black", fill="")

        for ball in engine.balls:
            x0 = ball.x - BALL_RADIUS
            y0 = ball.y - BALL_RADIUS
            x1 = ball.x + BALL_RADIUS
            y1 = ball.y + BALL_RADIUS
            canvas.create_oval(x0, y0, x1, y1, fill=ball.color)
            canvas.create_text(
                ball.x,
                ball.y,
                text=str(ball.number),
                angle=ball.spin_angle,
                font=("Arial", 8),
                fill="black",
            )

        root.after(int(delta_time * 1000), animate)

    animate()
    root.mainloop()


if __name__ == "__main__":
    main()
