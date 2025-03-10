import tkinter as tk
import math
import time
from dataclasses import dataclass
from typing import List, Tuple

CANVAS_SIZE = 800
HEPTAGON_RADIUS = 350  # Increased to ensure containment
BALL_RADIUS = 20
G = 0.8  # Gravity adjusted
FRICTION = 0.1  # Friction adjusted
SPIN_FRICTION = 0.95  # Spin damping
SPEED_ROTATION = 2 * math.pi / 5  # 360 degrees every 5 seconds in radians per second

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


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    color: str
    number: int
    spin: float


def main():
    root = tk.Tk()
    canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
    canvas.pack()

    center_x = CANVAS_SIZE // 2
    center_y = CANVAS_SIZE // 2

    rotation = 0.0

    balls = []
    for i in range(20):
        color = COLORS[i]
        ball = Ball(center_x, center_y, 0.0, 0.0, BALL_RADIUS, color, i + 1, 0.0)
        balls.append(ball)

    last_time = time.time()

    def update():
        nonlocal rotation, last_time
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time

        rotation += SPEED_ROTATION * delta_time

        for ball in balls:
            ax = -FRICTION * ball.vx
            ay = G - FRICTION * ball.vy

            ball.vx += ax * delta_time
            ball.vy += ay * delta_time

            ball.x += ball.vx * delta_time
            ball.y += ball.vy * delta_time
            ball.spin *= SPIN_FRICTION

        vertices = []
        for i in range(7):
            theta = (2 * math.pi * i) / 7 + rotation
            x = center_x + HEPTAGON_RADIUS * math.cos(theta)
            y = center_y + HEPTAGON_RADIUS * math.sin(theta)
            vertices.append((x, y))

        for ball in balls:
            min_dist = float("inf")
            closest_normal = (0, 0)
            for i in range(7):
                a = vertices[i]
                b = vertices[(i + 1) % 7]
                ax, ay = a
                bx, by = b
                dx_edge = bx - ax
                dy_edge = by - ay

                ap_x = ball.x - ax
                ap_y = ball.y - ay
                t = (ap_x * dx_edge + ap_y * dy_edge) / (dx_edge**2 + dy_edge**2 + 1e-6)
                t_clamped = max(0.0, min(1.0, t))
                closest_x = ax + dx_edge * t_clamped
                closest_y = ay + dy_edge * t_clamped

                dx = ball.x - closest_x
                dy = ball.y - closest_y
                dist = math.hypot(dx, dy)

                if dist < min_dist:
                    min_dist = dist
                    edge_dir_x = dx_edge
                    edge_dir_y = dy_edge
                    normal_x = edge_dir_y
                    normal_y = -edge_dir_x
                    norm = math.hypot(normal_x, normal_y)
                    if norm > 1e-6:
                        normal_x /= norm
                        normal_y /= norm
                    closest_normal = (normal_x, normal_y)

            if min_dist < ball.radius:
                nx, ny = closest_normal
                v = (ball.vx, ball.vy)
                dot = v[0] * nx + v[1] * ny
                ball.vx = v[0] - 2 * dot * nx
                ball.vy = v[1] - 2 * dot * ny

                displacement = ball.radius - min_dist
                ball.x += displacement * nx
                ball.y += displacement * ny

        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                ball1 = balls[i]
                ball2 = balls[j]
                dx = ball1.x - ball2.x
                dy = ball1.y - ball2.y
                dist = math.hypot(dx, dy)
                if dist < 2 * BALL_RADIUS + 1e-6:
                    dir_x = dx / dist
                    dir_y = dy / dist
                    v1 = (ball1.vx, ball1.vy)
                    v2 = (ball2.vx, ball2.vy)

                    v1_dot = v1[0] * dir_x + v1[1] * dir_y
                    v2_dot = v2[0] * dir_x + v2[1] * dir_y

                    imp = (v1_dot - v2_dot) * 2.0
                    imp /= 2.0

                    ball1.vx -= imp * dir_x
                    ball1.vy -= imp * dir_y
                    ball2.vx += imp * dir_x
                    ball2.vy += imp * dir_y

                    move_dist = (2 * BALL_RADIUS - dist) / 2.0
                    ball1.x += dir_x * move_dist
                    ball1.y += dir_y * move_dist
                    ball2.x -= dir_x * move_dist
                    ball2.y -= dir_y * move_dist

        canvas.delete("all")
        rotated_vertices = []
        for i in range(7):
            theta = (2 * math.pi * i) / 7 + rotation
            x = center_x + HEPTAGON_RADIUS * math.cos(theta)
            y = center_y + HEPTAGON_RADIUS * math.sin(theta)
            rotated_vertices.append((x, y))
        canvas.create_polygon(rotated_vertices, outline="black", fill="", width=2)

        for ball in balls:
            x, y = ball.x, ball.y
            canvas.create_oval(
                x - BALL_RADIUS,
                y - BALL_RADIUS,
                x + BALL_RADIUS,
                y + BALL_RADIUS,
                fill=ball.color,
                outline="black",
            )
            canvas.create_text(x, y, text=str(ball.number), fill="black")

        canvas.update()
        root.after(16, update)

    root.after(0, update)
    root.mainloop()


if __name__ == "__main__":
    main()
