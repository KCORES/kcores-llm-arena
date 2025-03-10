import tkinter as tk
import math
import random
import time


class Ball:
    def __init__(self, x, y, radius, color, number):
        self.x = x
        self.y = y
        self.vx = 0.0
        self.vy = 0.0
        self.radius = radius
        self.color = color
        self.number = number
        self.angular_velocity = 0.0
        self.angle = 0.0


class Heptagon:
    def __init__(self, radius, rotation_speed):
        self.radius = radius
        self.rotation_speed = rotation_speed  # radians per second
        self.angle = 0.0  # current rotation angle

    def update(self, dt):
        self.angle += self.rotation_speed * dt
        self.angle %= 2 * math.pi

    def get_vertices(self):
        vertices = []
        for i in range(7):
            theta = self.angle + (2 * math.pi * i) / 7
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            vertices.append((x, y))
        return vertices


def closest_point_on_segment(A, B, P):
    ax, ay = A
    bx, by = B
    px, py = P
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    t = (apx * abx + apy * aby) / (abx**2 + aby**2 + 1e-8)
    if t < 0.0:
        return (ax, ay)
    elif t > 1.0:
        return (bx, by)
    return (ax + t * abx, ay + t * aby)


def compute_normal(A, B):
    ax, ay = A
    bx, by = B
    nx, ny = ay - by, bx - ax
    length = math.hypot(nx, ny)
    if length == 0:
        return (0, 0)
    return (nx / length, ny / length)


class Simulation:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=800, height=600)
        self.canvas.pack()

        self.heptagon = Heptagon(radius=300, rotation_speed=math.radians(360 / 5))
        self.ball_radius = 15
        self.colors = [
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

        self.balls = []
        for i in range(20):
            ball = Ball(0, 0, self.ball_radius, self.colors[i], i + 1)
            ball.vx = random.uniform(-100, 100)
            ball.vy = random.uniform(-100, 100)
            self.balls.append(ball)

        self.previous_time = time.time()
        self.canvas_heptagon = None
        self.ball_circles = []
        self.ball_texts = []
        self.init_drawing()
        self.master.after(16, self.update)

    def init_drawing(self):
        self.ball_circles = []
        self.ball_texts = []
        for ball in self.balls:
            x1 = 400 + ball.x - ball.radius
            y1 = 300 - ball.y - ball.radius
            x2 = 400 + ball.x + ball.radius
            y2 = 300 - ball.y + ball.radius
            circle = self.canvas.create_oval(
                x1, y1, x2, y2, fill=ball.color, outline=""
            )
            text_x = 400 + ball.x + ball.radius * math.cos(ball.angle)
            text_y = 300 - ball.y - ball.radius * math.sin(ball.angle)
            text = self.canvas.create_text(
                text_x, text_y, text=str(ball.number), fill="white"
            )
            self.ball_circles.append(circle)
            self.ball_texts.append(text)
        self.draw_heptagon()

    def draw_heptagon(self):
        if self.canvas_heptagon:
            self.canvas.delete(self.canvas_heptagon)
        vertices = self.heptagon.get_vertices()
        canvas_points = []
        for x, y in vertices:
            canvas_points.extend([400 + x, 300 - y])
        canvas_points.extend(canvas_points[0:2])
        self.canvas_heptagon = self.canvas.create_line(
            *canvas_points, fill="black", width=2
        )

    def update(self):
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        self.heptagon.update(dt)
        gravity = -800
        linear_friction = 0.2
        spin_friction = 0.95
        cor_wall = 0.7
        cor_ball = 0.8

        for ball in self.balls:
            ball.vy += gravity * dt
            ball.vx *= math.exp(-linear_friction * dt)
            ball.vy *= math.exp(-linear_friction * dt)
            ball.x += ball.vx * dt
            ball.y += ball.vy * dt

        vertices = self.heptagon.get_vertices()
        edges = [(vertices[i], vertices[(i + 1) % 7]) for i in range(7)]
        for ball in self.balls:
            for A, B in edges:
                closest = closest_point_on_segment(A, B, (ball.x, ball.y))
                dx, dy = ball.x - closest[0], ball.y - closest[1]
                distance = math.hypot(dx, dy)
                if distance < ball.radius:
                    nx, ny = compute_normal(A, B)
                    omega = self.heptagon.rotation_speed
                    v_wall_x = -omega * closest[1]
                    v_wall_y = omega * closest[0]
                    v_rel_x = ball.vx - v_wall_x
                    v_rel_y = ball.vy - v_wall_y
                    v_rel_dot_n = v_rel_x * nx + v_rel_y * ny
                    if v_rel_dot_n < 0:
                        j = -(1 + cor_wall) * v_rel_dot_n
                        ball.vx += j * nx
                        ball.vy += j * ny
                        penetration = ball.radius - distance
                        ball.x += nx * penetration
                        ball.y += ny * penetration
                        tangent_x = v_rel_x - v_rel_dot_n * nx
                        tangent_y = v_rel_y - v_rel_dot_n * ny
                        cross = nx * tangent_y - ny * tangent_x
                        ball.angular_velocity += (
                            (-1 if cross < 0 else 1)
                            * math.hypot(tangent_x, tangent_y)
                            / ball.radius
                            * dt
                        )

        for i in range(len(self.balls)):
            for j in range(i + 1, len(self.balls)):
                b1, b2 = self.balls[i], self.balls[j]
                dx, dy = b1.x - b2.x, b1.y - b2.y
                distance = math.hypot(dx, dy)
                min_dist = b1.radius + b2.radius
                if distance < min_dist:
                    if distance == 0:
                        continue
                    nx, ny = dx / distance, dy / distance
                    v_rel_x, v_rel_y = b1.vx - b2.vx, b1.vy - b2.vy
                    v_rel_dot_n = v_rel_x * nx + v_rel_y * ny
                    if v_rel_dot_n < 0:
                        j = -(1 + cor_ball) * v_rel_dot_n / 2
                        b1.vx += j * nx
                        b1.vy += j * ny
                        b2.vx -= j * nx
                        b2.vy -= j * ny
                        correction = (min_dist - distance) * nx / 2, (
                            min_dist - distance
                        ) * ny / 2
                        b1.x += correction[0]
                        b1.y += correction[1]
                        b2.x -= correction[0]
                        b2.y -= correction[1]
                        tangent_x = v_rel_x - v_rel_dot_n * nx
                        tangent_y = v_rel_y - v_rel_dot_n * ny
                        b1.angular_velocity += (
                            math.hypot(tangent_x, tangent_y) / b1.radius * dt
                        )
                        b2.angular_velocity -= (
                            math.hypot(tangent_x, tangent_y) / b2.radius * dt
                        )

        for ball in self.balls:
            ball.angular_velocity *= math.pow(spin_friction, dt)
            ball.angle += ball.angular_velocity * dt
            ball.angle %= 2 * math.pi

        self.redraw()
        self.master.after(16, self.update)

    def redraw(self):
        self.draw_heptagon()
        for i, ball in enumerate(self.balls):
            x1 = 400 + ball.x - ball.radius
            y1 = 300 - ball.y - ball.radius
            x2 = 400 + ball.x + ball.radius
            y2 = 300 - ball.y + ball.radius
            self.canvas.coords(self.ball_circles[i], x1, y1, x2, y2)
            text_x = 400 + ball.x + ball.radius * math.cos(ball.angle)
            text_y = 300 - ball.y - ball.radius * math.sin(ball.angle)
            self.canvas.coords(self.ball_texts[i], text_x, text_y)
            self.canvas.itemconfig(self.ball_texts[i], text=str(ball.number))


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    Simulation(root)
    root.mainloop()
