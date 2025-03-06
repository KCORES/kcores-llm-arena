import tkinter as tk
import numpy as np
from math import cos, sin, pi
from dataclasses import dataclass

WIDTH, HEIGHT = 600, 600
HEPTAGON_RADIUS = 200  # Radius of the heptagon vertices.
gravity_acceleration = 1  # pixels per second squared downward.
friction = 0.99  # Velocity damping each frame.
ball_radius = 10  # Each ball's radius.
colors = [
    "#f8b862",
    "#f6ad49",
    "#f39800",
    "#f08300",
    "#ec6d51",
    "#ee7948",
    "#ed6d3d",
    "#ec6800",
    "#ee7800",
    "#eb6238",
    "#ea5506",
    "#eb6101",
    "#e49e61",
    "#e45e32",
    "#e17b34",
    "#dd7a56",
    "#db8449",
    "#d66a35",
]


class Ball:
    def __init__(self, pos, vel):
        self.pos = np.array(pos)
        self.vel = np.array(vel)
        self.radius = ball_radius


def calculate_heptagon_vertices(theta):
    vertices = []
    angle_step = 2 * pi / 7  # radians per each vertex step.
    for k in range(7):
        angle = k * angle_step + theta
        x = HEPTAGON_RADIUS * cos(angle)
        y = HEPTAGON_RADIUS * sin(angle)
        vertices.append((x + 300, y + 300))  # Canvas center is 300,300.
    return vertices


def calculate_normals(vertices):
    normals = []
    for i in range(7):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % 7]
        edge_dir = np.array([x2 - x1, y2 - y1])
        normal_dir = np.array([edge_dir[1], -edge_dir[0]])  # 90 clockwise rotation
        normal_unit = normal_dir / np.linalg.norm(normal_dir)
        normals.append(normal_unit)
    return normals


def main():
    root = tk.Tk()
    root.title("Bouncing Balls")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    theta = 0
    angular_speed_rad_per_sec = 2 * pi / 5  # 360 degrees per 5 seconds.

    balls = []
    initial_speed = 10  # pixels/second initial speed for balls.
    for _ in range(20):
        angle = 2 * pi * np.random.rand()
        vx = initial_speed * cos(angle)
        vy = initial_speed * sin(angle)
        balls.append(Ball(np.array([300, 300]), np.array([vx, vy])))

    def update():
        nonlocal theta

        dt = 0.1  # Time step in seconds.
        theta += angular_speed_rad_per_sec * dt

        vertices = calculate_heptagon_vertices(theta)
        edge_normals = calculate_normals(vertices)

        # Process each ball for movement and collision.
        for ball in balls:
            # Apply gravity and friction.
            ball.pos += ball.vel * dt
            ball.vel[1] += gravity_acceleration * dt  # Gravity downward.
            ball.vel *= friction  # Damping.

            # Ball-edge collision checks.
            for i in range(7):
                edge_start = np.array(vertices[i])
                edge_end = np.array(vertices[(i + 1) % 7])
                edge_vector = edge_end - edge_start

                # Ball to edge_start vector.
                pos_vec = ball.pos - edge_start
                distance_along_edge = np.dot(pos_vec, edge_vector) / np.linalg.norm(
                    edge_vector
                )
                distance_perp = np.linalg.norm(
                    np.cross(edge_vector, pos_vec)
                ) / np.linalg.norm(edge_vector)

                if distance_perp > ball_radius:
                    normal_unit = edge_normals[i]
                    normal_component = np.dot(ball.vel, normal_unit)
                    ball.vel -= 2 * normal_component * normal_unit

        # Ball-ball collisions.
        for i in range(len(balls)):
            for j in range(i + 1, len(balls)):
                ball_a, ball_b = balls[i], balls[j]
                dx = ball_a.pos[0] - ball_b.pos[0]
                dy = ball_a.pos[1] - ball_b.pos[1]
                distance_sq = dx * dx + dy * dy
                if distance_sq < (2 * ball_radius) ** 2:
                    normal_dir = np.array([dx, dy]) / np.linalg.norm([dx, dy])
                    va_dot_n = np.dot(ball_a.vel, normal_dir)
                    vb_dot_n = np.dot(ball_b.vel, normal_dir)
                    ball_a.vel -= 2 * va_dot_n * normal_dir
                    ball_b.vel -= 2 * vb_dot_n * normal_dir

        # Clear and redraw.
        canvas.delete("all")
        # Draw heptagon edges.
        for k in range(7):
            x1, y1 = vertices[k]
            x2, y2 = vertices[(k + 1) % 7]
            canvas.create_line(x1, y1, x2, y2)
        # Draw balls.
        for ball in balls:
            x, y = ball.pos[0], ball.pos[1]
            canvas.create_oval(
                x - ball_radius,
                y - ball_radius,
                x + ball_radius,
                y + ball_radius,
                fill=colors[len(balls) - 1 % len(colors)],
            )
        root.after(16, update)  # ~60 fps

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
