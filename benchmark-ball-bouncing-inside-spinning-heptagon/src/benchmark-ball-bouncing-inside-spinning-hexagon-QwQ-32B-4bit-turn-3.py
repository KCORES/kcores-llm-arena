import tkinter as tk
import math
import random
from dataclasses import dataclass


@dataclass
class Ball:
    x: float
    y: float
    vx: float
    vy: float
    omega: float
    radius: float
    number: int
    color: str
    angle: float = 0.0


def main():
    canvas_width = 600
    canvas_height = 600
    root = tk.Tk()
    root.title("Bouncing Balls in Rotating Heptagon")
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg="white")
    canvas.pack()

    center_x = canvas_width / 2
    center_y = canvas_height / 2
    radius_heptagon = 250
    radius_ball = 10
    gravity = 0.5
    friction = 0.95
    angular_friction = 0.95
    initial_speed = 5
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

    theta = 0
    delta_theta = math.radians(1.2)  # 1.2 degrees per frame

    balls = []
    for i in range(20):
        angle = random.uniform(0, 2 * math.pi)
        vx = math.cos(angle) * initial_speed
        vy = math.sin(angle) * initial_speed
        color = colors[i]
        balls.append(
            Ball(
                x=center_x,
                y=center_y,
                vx=vx,
                vy=vy,
                omega=0.0,
                radius=radius_ball,
                number=i + 1,
                color=color,
            )
        )

    def get_heptagon_vertices():
        vertices = []
        for i in range(7):
            angle = theta + (2 * math.pi / 7) * i
            x = center_x + radius_heptagon * math.cos(angle)
            y = center_y + radius_heptagon * math.sin(angle)
            vertices.append((x, y))
        return vertices

    def handle_wall_collisions(ball):
        poly_vertices = get_heptagon_vertices()
        min_dist = float("inf")
        closest_point = (0, 0)
        best_n = (0, 0)
        for i in range(len(poly_vertices)):
            A = poly_vertices[i]
            B = poly_vertices[(i + 1) % 7]
            ax, ay = A
            bx, by = B
            abx = bx - ax
            aby = by - ay
            apx = ball.x - ax
            apy = ball.y - ay
            t = (apx * abx + apy * aby) / (abx**2 + aby**2)
            t = max(0.0, min(1.0, t))
            px = ax + t * abx
            py = ay + t * aby
            dx = ball.x - px
            dy = ball.y - py
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest_point = (px, py)
                nx = dx
                ny = dy
                n_len = math.hypot(nx, ny)
                if n_len < 1e-6:
                    nx, ny = 0, 0
                else:
                    nx /= n_len
                    ny /= n_len
                best_n = (nx, ny)
        if min_dist < ball.radius + 1e-6:
            px, py = closest_point
            nx, ny = best_n
            d = min_dist
            scale = (d - ball.radius) / d
            ball.x = ball.x - (ball.x - px) * scale
            ball.y = ball.y - (ball.y - py) * scale
            v_dot_n = ball.vx * nx + ball.vy * ny
            ball.vx -= 2 * v_dot_n * nx
            ball.vy -= 2 * v_dot_n * ny

    def handle_ball_collision(b1, b2):
        dx = b1.x - b2.x
        dy = b1.y - b2.y
        dist_sq = dx**2 + dy**2
        if dist_sq < 1e-6:
            return
        dist = math.sqrt(dist_sq)
        if dist < 2 * b1.radius:
            nx = dx / dist
            ny = dy / dist
            v1x, v1y = b1.vx, b1.vy
            v2x, v2y = b2.vx, b2.vy
            dvx = v1x - v2x
            dvy = v1y - v2y
            dot = dvx * nx + dvy * ny
            j = -2 * dot
            b1.vx -= j * nx
            b1.vy -= j * ny
            b2.vx += j * nx
            b2.vy += j * ny

    def animate():
        nonlocal theta
        theta += delta_theta
        poly_vertices = get_heptagon_vertices()
        for ball in balls:
            ball.vy += gravity
            ball.vx *= friction
            ball.vy *= friction
            ball.omega *= angular_friction
            ball.x += ball.vx
            ball.y += ball.vy
            handle_wall_collisions(ball)
            ball.angle += ball.omega  # Update rotation angle
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                handle_ball_collision(balls[i], balls[j])
        canvas.delete("all")
        coords = []
        for v in poly_vertices:
            coords.extend(v)
        canvas.create_polygon(coords, outline="black", fill="")
        for ball in balls:
            x, y = ball.x, ball.y
            r = ball.radius
            canvas.create_oval(
                x - r, y - r, x + r, y + r, fill=ball.color, outline="black"
            )
            canvas.create_text(x, y, text=str(ball.number), fill="black")
        canvas.update()
        root.after(16, animate)

    root.after(0, animate)
    root.mainloop()


if __name__ == "__main__":
    main()
