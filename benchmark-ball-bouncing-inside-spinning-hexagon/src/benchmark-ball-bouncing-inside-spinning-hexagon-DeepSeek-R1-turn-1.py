import tkinter as tk
import math
import random
import time

# Constants
WIDTH, HEIGHT = 800, 800
CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2
BALL_RADIUS = 12
HEPTAGON_RADIUS = 280
GRAVITY = 800
AIR_FRICTION = 0.1
SPIN_FRICTION = 0.1
RESTITUTION = 0.8
FRICTION_TANGENT = 0.3
ROTATION_SPEED = 72  # Degrees per second
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

# Precompute heptagon collision data
HEPTAGON_EDGES = []
n = 7
for k in range(n):
    theta1 = 2 * math.pi * k / n
    theta2 = 2 * math.pi * (k + 1) / n
    v1 = (math.cos(theta1), math.sin(theta1))
    v2 = (math.cos(theta2), math.sin(theta2))
    mid = ((v1[0] + v2[0]) / 2, (v1[1] + v2[1]) / 2)
    length = math.hypot(mid[0], mid[1])
    normal = (-mid[0] / length, -mid[1] / length)
    HEPTAGON_EDGES.append((v1, v2, normal))


class Heptagon:
    def __init__(self):
        self.angle = 0  # Degrees
        self.rotation_speed = ROTATION_SPEED

    def update(self, dt):
        self.angle += self.rotation_speed * dt
        self.angle %= 360

    def draw(self, canvas):
        angle_rad = math.radians(self.angle)
        points = []
        for k in range(7):
            theta = 2 * math.pi * k / 7 + angle_rad
            x = CENTER_X + HEPTAGON_RADIUS * math.cos(theta)
            y = CENTER_Y + HEPTAGON_RADIUS * math.sin(theta)
            points.extend([x, y])
        canvas.create_polygon(points, outline="black", fill="", width=2)


class Ball:
    def __init__(self, x, y, color, number):
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.radius = BALL_RADIUS
        self.color = color
        self.number = number
        self.omega = random.uniform(-180, 180)  # Degrees per second
        self.rotation = 0  # Degrees

    def update(self, dt):
        # Apply physics
        self.vy += GRAVITY * dt
        self.vx *= 1 - AIR_FRICTION * dt
        self.vy *= 1 - AIR_FRICTION * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.omega *= 1 - SPIN_FRICTION * dt
        self.rotation += self.omega * dt
        self.rotation %= 360

    def draw(self, canvas):
        # Draw ball
        canvas.create_oval(
            self.x - self.radius,
            self.y - self.radius,
            self.x + self.radius,
            self.y + self.radius,
            fill=self.color,
            outline="black",
        )
        # Draw rotating number
        angle_rad = math.radians(self.rotation)
        tx = self.x + self.radius * math.cos(angle_rad)
        ty = self.y + self.radius * math.sin(angle_rad)
        canvas.create_text(tx, ty, text=str(self.number), fill="black")


class BouncingBalls:
    def __init__(self, master):
        self.master = master
        self.canvas = tk.Canvas(master, width=WIDTH, height=HEIGHT, bg="white")
        self.canvas.pack()

        self.heptagon = Heptagon()
        self.balls = []
        for i in range(20):
            ball = Ball(CENTER_X, CENTER_Y, BALL_COLORS[i], i + 1)
            angle = random.uniform(0, 2 * math.pi)
            speed = random.uniform(50, 150)
            ball.vx = math.cos(angle) * speed
            ball.vy = math.sin(angle) * speed
            self.balls.append(ball)

        self.previous_time = time.time()
        self.run()

    def run(self):
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        self.heptagon.update(dt)
        for ball in self.balls:
            ball.update(dt)

        self.handle_boundary_collisions()
        self.handle_ball_collisions()

        self.canvas.delete("all")
        self.heptagon.draw(self.canvas)
        for ball in self.balls:
            ball.draw(self.canvas)

        self.master.after(10, self.run)

    def handle_boundary_collisions(self):
        angle_rad = math.radians(self.heptagon.angle)
        cos_rot = math.cos(-angle_rad)
        sin_rot = math.sin(-angle_rad)

        for ball in self.balls:
            dx = ball.x - CENTER_X
            dy = ball.y - CENTER_Y
            tx = dx * cos_rot - dy * sin_rot
            ty = dx * sin_rot + dy * cos_rot

            min_dist = float("inf")
            collision_normal = None
            for v1, v2, normal in HEPTAGON_EDGES:
                edge_dist = (tx - v1[0] * HEPTAGON_RADIUS) * normal[0] + (
                    ty - v1[1] * HEPTAGON_RADIUS
                ) * normal[1]
                if edge_dist < min_dist:
                    min_dist = edge_dist
                    collision_normal = normal

            if min_dist < BALL_RADIUS:
                # Transform velocity to heptagon space
                vx_rot = ball.vx * cos_rot - ball.vy * sin_rot
                vy_rot = ball.vx * sin_rot + ball.vy * cos_rot

                # Collision response
                dot = vx_rot * collision_normal[0] + vy_rot * collision_normal[1]
                if dot < 0:
                    vx_rot -= (1 + RESTITUTION) * dot * collision_normal[0]
                    vy_rot -= (1 + RESTITUTION) * dot * collision_normal[1]

                    # Tangential friction
                    tangent = (-collision_normal[1], collision_normal[0])
                    vt = vx_rot * tangent[0] + vy_rot * tangent[1]
                    vt *= 1 - FRICTION_TANGENT
                    vx_rot = vt * tangent[0] + vx_rot
                    vy_rot = vt * tangent[1] + vy_rot

                # Transform back to world space
                ball.vx = vx_rot * cos_rot + vy_rot * sin_rot
                ball.vy = -vx_rot * sin_rot + vy_rot * cos_rot

                # Position correction
                pen = BALL_RADIUS - min_dist
                tx += collision_normal[0] * pen
                ty += collision_normal[1] * pen
                ball.x = tx * cos_rot + ty * sin_rot + CENTER_X
                ball.y = -tx * sin_rot + ty * cos_rot + CENTER_Y
                ball.omega += vt * 10

    def handle_ball_collisions(self):
        for i in range(len(self.balls)):
            a = self.balls[i]
            for j in range(i + 1, len(self.balls)):
                b = self.balls[j]
                dx = b.x - a.x
                dy = b.y - a.y
                dist = math.hypot(dx, dy)
                if dist < 2 * BALL_RADIUS and dist != 0:
                    nx = dx / dist
                    ny = dy / dist
                    tvx = a.vx - b.vx
                    tvy = a.vy - b.vy
                    dot = tvx * nx + tvy * ny
                    if dot > 0:
                        continue

                    imp = -(1 + RESTITUTION) * dot / 2
                    a.vx -= imp * nx
                    a.vy -= imp * ny
                    b.vx += imp * nx
                    b.vy += imp * ny

                    # Tangential friction
                    tangent = (-ny, nx)
                    vt = (a.vx - b.vx) * tangent[0] + (a.vy - b.vy) * tangent[1]
                    vt *= 1 - FRICTION_TANGENT
                    a.vx -= vt * tangent[0] / 2
                    a.vy -= vt * tangent[1] / 2
                    b.vx += vt * tangent[0] / 2
                    b.vy += vt * tangent[1] / 2

                    # Position correction
                    overlap = (2 * BALL_RADIUS - dist) / 2
                    a.x -= nx * overlap
                    a.y -= ny * overlap
                    b.x += nx * overlap
                    b.y += ny * overlap

                    a.omega += vt * 5
                    b.omega -= vt * 5


if __name__ == "__main__":
    root = tk.Tk()
    root.title("Bouncing Balls in Spinning Heptagon")
    app = BouncingBalls(root)
    root.mainloop()
