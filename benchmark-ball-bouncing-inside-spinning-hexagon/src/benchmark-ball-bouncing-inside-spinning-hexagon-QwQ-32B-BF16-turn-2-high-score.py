import tkinter as tk
import numpy as np
from math import cos, sin, pi

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


class Ball:
    def __init__(self, pos, velocity, radius, color, number):
        self.pos = np.array(pos, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.radius = radius
        self.color = color
        self.number = number
        self.angular_velocity = 0.0  # radians per second
        self.angle = 0.0  # current rotation angle in radians


def main():
    root = tk.Tk()
    canvas_width = 800
    canvas_height = 800
    canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
    canvas.pack()

    center_x = canvas_width / 2
    center_y = canvas_height / 2
    heptagon_radius = 300
    ball_radius = 20

    balls = []
    for i in range(20):
        color = colors[i]
        initial_pos = [center_x, center_y]
        initial_velocity = [0.0, 0.0]
        balls.append(Ball(initial_pos, initial_velocity, ball_radius, color, i + 1))

    current_theta = 0.0
    angular_velocity_radians_per_sec = 2 * pi / 5  # 360 degrees per 5 seconds
    gravity = 9.8  # pixels/(s^2)
    damping_factor = 0.99  # velocity damping factor per second
    spin_damping = 0.98  # angular velocity damping per second
    COR_wall = 0.7
    delta_time = 1 / 60

    def update():
        nonlocal current_theta
        current_theta += angular_velocity_radians_per_sec * delta_time

        vertices = []
        angle_step = 2 * pi / 7
        for i in range(7):
            angle = current_theta + i * angle_step
            x = center_x + heptagon_radius * cos(angle)
            y = center_y + heptagon_radius * sin(angle)
            vertices.append((x, y))
        vertices.append(vertices[0])
        edges = [(vertices[i], vertices[i + 1]) for i in range(7)]

        for ball in balls:
            ball.velocity[1] += gravity * delta_time
            ball.velocity *= 1 - (1 - damping_factor) * delta_time
            ball.pos += ball.velocity * delta_time

            collision_occurred = False
            for edge in edges:
                (x1, y1), (x2, y2) = edge
                A = np.array([x1, y1])
                B = np.array([x2, y2])
                AP = ball.pos - A
                AB = B - A
                t = np.dot(AP, AB) / np.dot(AB, AB)
                t_clamped = np.clip(t, 0.0, 1.0)
                closest_point = A + t_clamped * AB
                closest_vector = closest_point - ball.pos
                distance_sq = np.dot(closest_vector, closest_vector)

                if distance_sq <= (ball.radius) ** 2 + 1e-6:
                    edge_dir = AB
                    normal_dir = np.array([edge_dir[1], -edge_dir[0]])
                    length = np.linalg.norm(normal_dir)
                    if length == 0:
                        continue
                    n = normal_dir / length
                    v_dot_n = np.dot(ball.velocity, n)
                    delta_v = -(1 + COR_wall) * v_dot_n * n
                    ball.velocity = ball.velocity + delta_v

                    tangent = np.array([-n[1], n[0]])
                    v_tangent = np.dot(ball.velocity, tangent)
                    ball.angular_velocity = v_tangent / ball.radius

                    distance = np.sqrt(distance_sq)
                    overlap = ball.radius - distance
                    move_dir = (closest_point - ball.pos) / distance
                    ball.pos += move_dir * overlap
                    collision_occurred = True
                    break

            if collision_occurred:
                continue

            for other in balls:
                if ball == other:
                    continue
                dx = other.pos[0] - ball.pos[0]
                dy = other.pos[1] - ball.pos[1]
                distance_sq = dx * dx + dy * dy
                if distance_sq < (2 * ball.radius) ** 2 + 1e-6:
                    distance = np.sqrt(distance_sq)
                    unit_normal = np.array([dx, dy]) / distance
                    v1, v2 = ball.velocity, other.velocity
                    delta_v = ((v1 - v2).dot(unit_normal)) * unit_normal
                    ball.velocity -= delta_v
                    other.velocity += delta_v

                    overlap = (2 * ball.radius - distance) / 2
                    ball.pos -= unit_normal * overlap
                    other.pos += unit_normal * overlap

                    tangent = np.array([-unit_normal[1], unit_normal[0]])
                    v_tang_ball = v1.dot(tangent)
                    v_tang_other = v2.dot(tangent)
                    ball.angular_velocity = v_tang_ball / ball.radius
                    other.angular_velocity = v_tang_other / other.radius

        for ball in balls:
            ball.angular_velocity *= spin_damping
            ball.angle += ball.angular_velocity * delta_time

        canvas.delete("all")
        for i in range(7):
            edge = edges[i]
            canvas.create_line(
                edge[0][0], edge[0][1], edge[1][0], edge[1][1], fill="black"
            )

        for ball in balls:
            x, y = ball.pos
            canvas.create_oval(
                x - ball.radius,
                y - ball.radius,
                x + ball.radius,
                y + ball.radius,
                fill=ball.color,
            )
            current_angle = np.degrees(ball.angle)
            canvas.create_text(x, y, text=str(ball.number), angle=current_angle)

        canvas.after(int(delta_time * 1000), update)

    update()
    root.mainloop()


if __name__ == "__main__":
    main()
