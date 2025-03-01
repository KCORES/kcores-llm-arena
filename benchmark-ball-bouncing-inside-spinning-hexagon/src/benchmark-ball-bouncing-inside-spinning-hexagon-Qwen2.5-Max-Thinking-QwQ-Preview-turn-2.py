import tkinter as tk
import math
from dataclasses import dataclass

WIDTH = 800
HEIGHT = 800
CENTER_X = WIDTH / 2
CENTER_Y = HEIGHT / 2

BALL_RADIUS = 20
HEPTAGON_RADIUS = 202  # Calculated to ensure balls fit within the heptagon

GRAVITY = 0.1
FRICTION = 0.99  # Air friction per frame
RESTITUTION_WALL = 0.8
RESTITUTION_BALL = 0.9
SPIN_FACTOR = 0.1  # Affects angular velocity based on tangential speed

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
    color: str
    number: int
    angle: float = 0.0
    angular_velocity: float = 0.0


balls = []
for i in range(20):
    balls.append(
        Ball(x=CENTER_X, y=CENTER_Y, vx=0, vy=0, color=COLORS[i], number=i + 1)
    )

rotation_angle = 0.0  # Degrees
rotation_speed = 72  # Degrees per second

root = tk.Tk()
canvas = tk.Canvas(root, width=WIDTH, height=HEIGHT)
canvas.pack()


def animate():
    global rotation_angle

    dt = 1 / 60  # 60 FPS
    rotation_angle += rotation_speed * dt
    rotation_angle %= 360
    theta_rad = math.radians(rotation_angle)
    omega = math.radians(rotation_speed)

    for ball in balls:
        ball.vy += GRAVITY
        ball.vx *= FRICTION
        ball.vy *= FRICTION
        ball.x += ball.vx
        ball.y += ball.vy

    local_edges = []
    for i in range(7):
        angle_deg = i * (360 / 7)
        angle_rad = math.radians(angle_deg)
        x = HEPTAGON_RADIUS * math.cos(angle_rad)
        y = HEPTAGON_RADIUS * math.sin(angle_rad)
        local_edges.append((x, y))
    local_edges.append(local_edges[0])

    for ball in balls:
        dx = ball.x - CENTER_X
        dy = ball.y - CENTER_Y
        x_local = dx * math.cos(-theta_rad) - dy * math.sin(-theta_rad)
        y_local = dx * math.sin(-theta_rad) + dy * math.cos(-theta_rad)

        inside = True
        for i in range(7):
            a, b = local_edges[i], local_edges[i + 1]
            ax, ay = a
            bx, by = b
            edge_x = bx - ax
            edge_y = by - ay
            if edge_x == 0 and edge_y == 0:
                continue
            length = math.hypot(edge_x, edge_y)
            nx = -edge_y / length
            ny = edge_x / length
            px = x_local - ax
            py = y_local - ay
            dot = px * nx + py * ny
            if dot < 0:
                inside = False
                break

        if not inside:
            min_dist = float("inf")
            closest_normal = None
            closest_point = None

            for i in range(7):
                a, b = local_edges[i], local_edges[i + 1]
                ax, ay = a
                bx, by = b
                abx = bx - ax
                aby = by - ay
                apx = x_local - ax
                apy = y_local - ay
                t = (
                    (apx * abx + apy * aby) / (abx**2 + aby**2)
                    if (abx**2 + aby**2) != 0
                    else 0
                )
                t = max(0, min(1, t))
                cx = ax + t * abx
                cy = ay + t * aby
                dx = x_local - cx
                dy = y_local - cy
                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    min_dist = dist
                    edge_dir_x = abx
                    edge_dir_y = aby
                    closest_point = (cx, cy)
                    length = math.hypot(edge_dir_x, edge_dir_y)
                    if length == 0:
                        continue
                    nx = -edge_dir_y / length
                    ny = edge_dir_x / length
                    closest_normal = (nx, ny)

            if min_dist < BALL_RADIUS:
                vrx = ball.vx + omega * y_local
                vry = ball.vy - omega * x_local

                vn = vrx * closest_normal[0] + vry * closest_normal[1]
                if vn < 0:
                    e = RESTITUTION_WALL
                    vrn_new = -vn * e
                    delta_vn = vrn_new - vn
                    vrx += delta_vn * closest_normal[0]
                    vry += delta_vn * closest_normal[1]

                    vt = -vrx * closest_normal[1] + vry * closest_normal[0]
                    vt_new = vt * 0.9
                    delta_vt = vt_new - vt
                    vrx += delta_vt * (-closest_normal[1])
                    vry += delta_vt * closest_normal[0]

                    ball.angular_velocity += vt * SPIN_FACTOR

                    ball.vx = vrx - omega * y_local
                    ball.vy = vry + omega * x_local

                    overlap = BALL_RADIUS - min_dist
                    move_x = closest_normal[0] * overlap
                    move_y = closest_normal[1] * overlap
                    ball.x += move_x * math.cos(theta_rad) - move_y * math.sin(
                        theta_rad
                    )
                    ball.y += move_x * math.sin(theta_rad) + move_y * math.cos(
                        theta_rad
                    )

    for i in range(len(balls)):
        for j in range(i + 1, len(balls)):
            ball1, ball2 = balls[i], balls[j]
            dx = ball2.x - ball1.x
            dy = ball2.y - ball1.y
            dist_sq = dx**2 + dy**2
            if dist_sq >= (2 * BALL_RADIUS) ** 2:
                continue

            dist = math.sqrt(dist_sq)
            nx = dx / dist
            ny = dy / dist

            rvx = ball2.vx - ball1.vx
            rvy = ball2.vy - ball1.vy
            vel_along_normal = rvx * nx + rvy * ny
            if vel_along_normal > 0:
                continue

            e = RESTITUTION_BALL
            j = -(1 + e) * vel_along_normal / 2.0
            impulse = j / 2.0

            ball1.vx -= impulse * nx
            ball1.vy -= impulse * ny
            ball2.vx += impulse * nx
            ball2.vy += impulse * ny

            tangent_x = -ny
            tangent_y = nx
            vel_along_tangent = rvx * tangent_x + rvy * tangent_y
            friction = 0.1
            ball1.vx -= vel_along_tangent * tangent_x * friction
            ball1.vy -= vel_along_tangent * tangent_y * friction
            ball2.vx += vel_along_tangent * tangent_x * friction
            ball2.vy += vel_along_tangent * tangent_y * friction

            ball1.angular_velocity += vel_along_tangent * SPIN_FACTOR
            ball2.angular_velocity -= vel_along_tangent * SPIN_FACTOR

            overlap = (2 * BALL_RADIUS - dist) / 2
            ball1.x -= nx * overlap
            ball1.y -= ny * overlap
            ball2.x += nx * overlap
            ball2.y += ny * overlap

    for ball in balls:
        ball.angular_velocity *= FRICTION
        ball.angle += ball.angular_velocity * dt

    canvas.delete("all")

    rotated_vertices = []
    for i in range(7):
        angle_deg = i * (360 / 7) + rotation_angle
        angle_rad = math.radians(angle_deg)
        x = CENTER_X + HEPTAGON_RADIUS * math.cos(angle_rad)
        y = CENTER_Y + HEPTAGON_RADIUS * math.sin(angle_rad)
        rotated_vertices.append((x, y))
    canvas.create_polygon(rotated_vertices, outline="black", fill="", width=2)

    for ball in balls:
        x1 = ball.x - BALL_RADIUS
        y1 = ball.y - BALL_RADIUS
        x2 = ball.x + BALL_RADIUS
        y2 = ball.y + BALL_RADIUS
        canvas.create_oval(x1, y1, x2, y2, fill=ball.color, outline="black")
        canvas.create_text(
            ball.x,
            ball.y,
            text=str(ball.number),
            angle=math.degrees(ball.angle),
            fill="black",
        )

    root.after(int(dt * 1000), animate)


animate()
root.mainloop()
