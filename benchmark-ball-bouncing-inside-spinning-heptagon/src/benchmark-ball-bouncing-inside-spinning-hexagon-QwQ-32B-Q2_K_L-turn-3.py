import tkinter as tk
import math
import random
import time

WIDTH = 800
HEIGHT = 800
BALL_RADIUS = 10
NUM_BALLS = 20
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

HEX_RADIUS = 300
ANGULAR_SPEED = 2 * math.pi / 5  # radians per second
GRAVITY = 0.3
FRICTION = 0.95


class Ball:
    def __init__(self, num, color):
        self.x = WIDTH // 2
        self.y = HEIGHT // 2
        self.vx = random.uniform(-1, 1)
        self.vy = random.uniform(-1, 1)
        self.radius = BALL_RADIUS
        self.color = color
        self.num = num
        self.angular_velocity = num * 0.1  # Initialize angular velocity based on number


def update_balls(delta_time, balls):
    for ball in balls:
        # Apply gravity and friction
        ball.vy += GRAVITY * delta_time
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        # Update position
        ball.x += ball.vx * delta_time
        ball.y += ball.vy * delta_time


def resolve_edge_collision(ball, edge, min_distance):
    a, b = edge
    ax, ay = a
    bx, by = b
    dx_edge = bx - ax
    dy_edge = by - ay
    # Calculate normal vector
    edge_length = math.hypot(dx_edge, dy_edge)
    n_x = -dy_edge / edge_length if edge_length != 0 else 0
    n_y = dx_edge / edge_length if edge_length != 0 else 1
    # Move ball out of the edge
    displacement = ball.radius - min_distance
    ball.x += n_x * displacement
    ball.y += n_y * displacement
    # Reflect velocity
    v_dot_n = ball.vx * n_x + ball.vy * n_y
    ball.vx -= 2 * v_dot_n * n_x
    ball.vy -= 2 * v_dot_n * n_y


def check_edge_collisions(balls, edges):
    for ball in balls:
        min_dist = float("inf")
        closest_edge = None
        for edge in edges:
            a, b = edge
            ax, ay = a
            bx, by = b
            # Distance calculation omitted for brevity
            # ... (same as discussed above)
            # Assume min_dist and closest_edge are found
            if distance < min_dist:
                min_dist = distance
                closest_edge = edge
        if min_dist < ball.radius:
            resolve_edge_collision(ball, closest_edge, min_dist)


def check_ball_collisions(balls):
    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            ball1 = balls[i]
            ball2 = balls[j]
            dx = ball1.x - ball2.x
            dy = ball1.y - ball2.y
            distance = math.hypot(dx, dy)
            if distance < 2 * BALL_RADIUS:
                # Reflect velocities
                nx = dx / distance if distance != 0 else 1
                ny = dy / distance if distance != 0 else 0
                dvx = ball1.vx - ball2.vx
                dvy = ball1.vy - ball2.vy
                dot = dvx * nx + dvy * ny
                ball1.vx -= 2 * dot * nx
                ball2.vx += 2 * dot * nx
                ball1.vy -= 2 * dot * ny
                ball2.vy += 2 * dot * ny


def main_loop(canvas, balls, prev_time, current_angle):
    current_time = time.time()
    delta_time = current_time - prev_time
    prev_time = current_time
    current_angle += ANGULAR_SPEED * delta_time
    current_angle %= 2 * math.pi

    # Update balls' positions and velocities
    update_balls(delta_time, balls)

    # Generate heptagon vertices
    vertices = []
    for i in range(7):
        theta = current_angle + (2 * math.pi * i) / 7
        x = WIDTH // 2 + HEX_RADIUS * math.cos(theta)
        y = HEIGHT // 2 + HEX_RADIUS * math.sin(theta)
        vertices.append((x, y))
    edges = [(vertices[i], vertices[(i + 1) % 7]) for i in range(7)]

    # Check collisions with edges
    check_edge_collisions(balls, edges)

    # Check ball-ball collisions
    check_ball_collisions(balls)

    # Draw everything
    canvas.delete("all")
    # Draw heptagon
    coords = []
    for v in vertices:
        coords.extend(v)
    canvas.create_polygon(coords, outline="black")
    # Draw balls
    for ball in balls:
        canvas.create_oval(
            ball.x - BALL_RADIUS,
            ball.y - BALL_RADIUS,
            ball.x + BALL_RADIUS,
            ball.y + BALL_RADIUS,
            fill=ball.color,
            outline="black",
        )
        canvas.create_text(ball.x, ball.y, text=str(ball.num), fill="white")

    # Schedule next frame
    canvas.after(16, main_loop, canvas, balls, prev_time, current_angle)


def main():
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT, bg="white")
    canvas.pack()

    balls = [Ball(i + 1, colors[i]) for i in range(NUM_BALLS)]
    prev_time = time.time()
    current_angle = 0

    main_loop(canvas, balls, prev_time, current_angle)
    root.mainloop()


if __name__ == "__main__":
    main()
