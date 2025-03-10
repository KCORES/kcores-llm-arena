import tkinter as tk
import math
import time
import random
from dataclasses import dataclass
import numpy as np

gravity = 0.5
friction = 0.1
ball_radius = 10
heptagon_radius = 250
canvas_width = 600
canvas_height = 600
center_x = canvas_width // 2
center_y = canvas_height // 2
angular_velocity = 2 * math.pi / 5  # 360 degrees per 5 seconds

colors = [
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
    spin_speed: float
    orientation: float
    color: str
    number: int
    radius = ball_radius  # class attribute


def create_balls():
    balls = []
    for i in range(20):
        color = colors[i]
        spin_speed = random.uniform(-1, 1) * math.pi
        ball = Ball(center_x, center_y, 0.0, 0.0, spin_speed, 0.0, color, i + 1)
        balls.append(ball)
    return balls


root = tk.Tk()
canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
canvas.pack()

rotation_angle = 0.0
previous_time = None
balls = create_balls()


def update():
    global rotation_angle, previous_time, balls

    current_time = time.time()
    if previous_time is None:
        previous_time = current_time
        root.after(10, update)
        return

    delta_time = current_time - previous_time
    previous_time = current_time

    rotation_angle += angular_velocity * delta_time

    vertices = []
    for i in range(7):
        angle = (2 * math.pi * i / 7) + rotation_angle
        x = center_x + heptagon_radius * math.cos(angle)
        y = center_y + heptagon_radius * math.sin(angle)
        vertices.append((x, y))

    for ball in balls:
        ball.vy += gravity * delta_time
        ball.vx *= 1 - friction * delta_time
        ball.vy *= 1 - friction * delta_time

        ball.x += ball.vx * delta_time
        ball.y += ball.vy * delta_time

        edges = []
        for i in range(len(vertices)):
            v1 = vertices[i]
            v2 = vertices[(i + 1) % len(vertices)]
            edges.append((v1, v2))

        collided = False
        for edge in edges:
            v1, v2 = edge
            x1, y1 = v1
            x2, y2 = v2
            dx_edge = x2 - x1
            dy_edge = y2 - y1

            bx = ball.x - x1
            by = ball.y - y1

            dot = bx * dx_edge + by * dy_edge

            if dot < 0:
                closest_x, closest_y = x1, y1
            else:
                len_sq = dx_edge**2 + dy_edge**2
                if dot > len_sq:
                    closest_x, closest_y = x2, y2
                else:
                    t = dot / len_sq
                    closest_x = x1 + t * dx_edge
                    closest_y = y1 + t * dy_edge

            dx_p = ball.x - closest_x
            dy_p = ball.y - closest_y
            dist = math.hypot(dx_p, dy_p)

            if dist < ball.radius:
                edge_dir_x = x2 - x1
                edge_dir_y = y2 - y1
                nx = edge_dir_y
                ny = -edge_dir_x
                len_n = math.hypot(nx, ny)
                if len_n == 0:
                    continue
                nx /= len_n
                ny /= len_n

                vx_wall = -angular_velocity * (closest_y - center_y)
                vy_wall = angular_velocity * (closest_x - center_x)
                wall_velocity = np.array([vx_wall, vy_wall])
                ball_velocity = np.array([ball.vx, ball.vy])
                relative_v = ball_velocity - wall_velocity

                normal = np.array([nx, ny])
                dot_vn = np.dot(relative_v, normal)
                reflected_relative = relative_v - 2 * dot_vn * normal

                new_ball_velocity = wall_velocity + reflected_relative
                ball.vx, ball.vy = new_ball_velocity

                direction = np.array([dx_p, dy_p]) / dist
                direction *= -1
                ball.x += direction[0] * ball.radius
                ball.y += direction[1] * ball.radius

                collided = True
                break

        if not collided:
            for other_ball in balls:
                if ball == other_ball:
                    continue
                dx = ball.x - other_ball.x
                dy = ball.y - other_ball.y
                dist_sq = dx**2 + dy**2
                if 0 < dist_sq < (2 * ball_radius) ** 2:
                    dist = math.sqrt(dist_sq)
                    nx = dx / dist
                    ny = dy / dist
                    unit = np.array([nx, ny])
                    v1 = np.array([ball.vx, ball.vy])
                    v2 = np.array([other_ball.vx, other_ball.vy])
                    dv = v1 - v2
                    dot = np.dot(dv, unit)
                    delta_v = -2 * dot * unit
                    ball.vx = v1[0] + delta_v[0]
                    ball.vy = v1[1] + delta_v[1]
                    other_ball.vx = v2[0] - delta_v[0]
                    other_ball.vy = v2[1] - delta_v[1]

                    move_distance = (2 * ball_radius - dist) / 2
                    ball.x += unit[0] * move_distance
                    ball.y += unit[1] * move_distance
                    other_ball.x -= unit[0] * move_distance
                    other_ball.y -= unit[1] * move_distance

    for ball in balls:
        ball.orientation += ball.spin_speed * delta_time

    canvas.delete("all")
    for i in range(len(vertices)):
        v1 = vertices[i]
        v2 = vertices[(i + 1) % len(vertices)]
        canvas.create_line(v1[0], v1[1], v2[0], v2[1], fill="black")

    for ball in balls:
        x, y = ball.x, ball.y
        r = ball.radius
        canvas.create_oval(x - r, y - r, x + r, y + r, fill=ball.color)
        text_angle = math.degrees(ball.orientation)
        canvas.create_text(x, y, text=str(ball.number), angle=text_angle, fill="black")

    root.after(10, update)


if __name__ == "__main__":
    previous_time = None
    root.after(10, update)
    root.mainloop()
